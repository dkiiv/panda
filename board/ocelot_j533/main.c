// ********************* Includes *********************
#include "../config.h"
#include "libc.h"

#include "main_declarations.h"
#include "critical.h"
#include "faults.h"

#include "drivers/registers.h"
#include "drivers/interrupts.h"
#include "drivers/llcan.h"
#include "drivers/llgpio.h"
#include "drivers/adc.h"

#include "board.h"

#include "drivers/clock.h"
#include "drivers/timer.h"

#include "gpio.h"
#include "crc.h"

// uncomment for usb debugging via debug_console.py
#define TGW_USB
// #define DEBUG_CAN
#define DEBUG_CTRL

#ifdef TGW_USB
  #include "drivers/uart.h"
  #include "drivers/usb.h"
#else
  // no serial either
  void puts(const char *a) {
    UNUSED(a);
  }
  void puth(unsigned int i) {
    UNUSED(i);
  }
  void puth2(unsigned int i) {
    UNUSED(i);
  }
#endif

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}

#ifdef TGW_USB

#include "ocelot_j533/can.h"

// ********************* usb debugging *********************
// TODO: neuter this if we are not debugging
void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

int usb_cb_ep1_in(void *usbdata, int len, bool hardwired) {
  UNUSED(hardwired);
  CAN_FIFOMailBox_TypeDef *reply = (CAN_FIFOMailBox_TypeDef *)usbdata;
  int ilen = 0;
  while (ilen < MIN(len/0x10, 4) && can_pop(&can_rx_q, &reply[ilen])) {
    ilen++;
  }
  return ilen*0x10;
}
// send on serial, first byte to select the ring
void usb_cb_ep2_out(void *usbdata, int len, bool hardwired) {
  UNUSED(hardwired);
  uint8_t *usbdata8 = (uint8_t *)usbdata;
  uart_ring *ur = get_ring_by_number(usbdata8[0]);
  if ((len != 0) && (ur != NULL)) {
    if ((usbdata8[0] < 2U)) {
      for (int i = 1; i < len; i++) {
        while (!putc(ur, usbdata8[i])) {
          // wait
        }
      }
    }
  }
}
// send on CAN
void usb_cb_ep3_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out_complete() {
  if (can_tx_check_min_slots_free(MAX_CAN_MSGS_PER_BULK_TRANSFER)) {
    usb_outep3_resume_if_paused();
  }
}

void usb_cb_enumeration_complete() {
  puts("USB enumeration complete\n");
  is_enumerated = 1;
}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, bool hardwired) {
  UNUSED(hardwired);
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  switch (setup->b.bRequest) {
    // **** 0xd1: enter bootloader mode
    case 0xd1:
      // this allows reflashing of the bootstub
      // so it's blocked over wifi
      switch (setup->b.wValue.w) {
        case 0:
          // only allow bootloader entry on debug builds
          #ifdef ALLOW_DEBUG
            if (hardwired) {
              puts("-> entering bootloader\n");
              enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
              NVIC_SystemReset();
            }
          #endif
          break;
        case 1:
          puts("-> entering softloader\n");
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
          break;
        default:
          puts("Bootloader mode invalid\n");
          break;
      }
      break;
    // **** 0xd8: reset ST
    case 0xd8:
      NVIC_SystemReset();
      break;
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) {
        break;
      }
      // read
      while ((resp_len < MIN(setup->b.wLength.w, MAX_RESP_LEN)) &&
                         getc(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    // **** 0xf1: Clear CAN ring buffer.
    case 0xf1:
      if (setup->b.wValue.w == 0xFFFFU) {
        puts("Clearing CAN Rx queue\n");
        can_clear(&can_rx_q);
      } else if (setup->b.wValue.w < BUS_MAX) {
        puts("Clearing CAN Tx queue\n");
        can_clear(can_queues[setup->b.wValue.w]);
      } else {
        puts("Clearing CAN CAN ring buffer failed: wrong bus number\n");
      }
      break;
    // **** 0xf2: Clear UART ring buffer.
    case 0xf2:
      {
        uart_ring * rb = get_ring_by_number(setup->b.wValue.w);
        if (rb != NULL) {
          puts("Clearing UART queue.\n");
          clear_uart_buff(rb);
        }
        break;
      }
    default:
      puts("NO HANDLER ");
      puth(setup->b.bRequest);
      puts("\n");
      break;
  }
  return resp_len;
}

#endif

// ***************************** can port *****************************

// Volkswagen PQ Checksum
uint8_t volkswagen_pq_compute_checksum(uint8_t *dat, int len) {
  uint8_t checksum = 0U;
  for (int i = 0; i < len; i++) {
    checksum += dat[i];
  }
  return checksum & 0xFF;
}

#define CAN_UPDATE  0xF0 //bootloader
#define COUNTER_CYCLE 0xFU
uint8_t counter = 0;

void CAN1_TX_IRQ_Handler(void) {
  process_can(0);
}

void CAN2_TX_IRQ_Handler(void) {
  process_can(1);
}

void CAN3_TX_IRQ_Handler(void) {
  process_can(2);
}

#define FAULT_SCE 3U
#define STARTUP 4U
uint8_t state = STARTUP;

#define LED_PIN 6
uint8_t flash_led = 0;
int led_value = 0;

// Bus 0: Ext Can
// Bus 1: Car PTCan
// Bus 2: GW PTCan

//------------- BUS 0 - EXT CAN --------------//

bool send = 0;

//------------- BUS 1 - CAR PTCAN ------------//

//------------- BUS 2 - GW PTCAN -------------//

#define GRA_Neu 0x38A

//------------- CAN FWDing below -------------//

void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {

    //CAN_FIFOMailBox_TypeDef to_fwd;
    //to_fwd.RIR = CAN1->sFIFOMailBox[0].RIR; // TX disabled. to send, OR this with 1.
    //to_fwd.RDTR = CAN1->sFIFOMailBox[0].RDTR;
    //to_fwd.RDLR = CAN1->sFIFOMailBox[0].RDLR;
    //to_fwd.RDHR = CAN1->sFIFOMailBox[0].RDHR;

    //uint16_t address = CAN1->sFIFOMailBox[0].RIR >> 21;

    #ifdef DEBUG_CAN
    puts("CAN1 RX: ");
    puth(address);
    puts("\n");
    #endif

    // CAN data buffer
    // uint8_t dat[8];

    //switch (address) {
    //  case CAN_UPDATE:
    //    if (GET_BYTES_04(&CAN1->sFIFOMailBox[0]) == 0xdeadface) {
    //      if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x0ab00b1e) {
    //        enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
    //        NVIC_SystemReset();
    //      } else if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x02b00b1e) {
    //        enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
    //        NVIC_SystemReset();
    //      } else {
    //        puts("Failed entering Softloader or Bootloader\n");
    //      }
    //    }
    //    break;
    //  default:
    //    // FWD as-is
    //    break;
    //}
    // no forward, can 1 is injection
    // can_send(&to_fwd, 0, false);
    // next
    can_rx(0);
    // CAN1->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN1);
  llcan_clear_send(CAN1);
}

void CAN2_RX0_IRQ_Handler(void) {
  // to the gateway (PT)
  while ((CAN2->RF0R & CAN_RF0R_FMP0) != 0) {

    /*CAN_FIFOMailBox_TypeDef to_fwd;
    to_fwd.RIR = CAN2->sFIFOMailBox[0].RIR | 1; // TXQ
    to_fwd.RDTR = CAN2->sFIFOMailBox[0].RDTR;
    to_fwd.RDLR = CAN2->sFIFOMailBox[0].RDLR;
    to_fwd.RDHR = CAN2->sFIFOMailBox[0].RDHR;

    uint16_t address = CAN2->sFIFOMailBox[0].RIR >> 21;

    #ifdef DEBUG_CAN
    puts("CAN2 RX: ");
    puth(address);
    puts("\n");
    #endif

    // CAN data buffer
    uint8_t dat[8];

    switch(address) {
      default:
        // FWD as-is
        break;
    }
    // send to CAN3 with love from CAN2
    can_send(&to_fwd, 2, false);
    // next
    can_rx(1);*/
  }
}

void CAN2_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN2);
  llcan_clear_send(CAN2);
}

void CAN3_RX0_IRQ_Handler(void) {
  // From the gateway (PT)
  while ((CAN3->RF0R & CAN_RF0R_FMP0) != 0) {

    CAN_FIFOMailBox_TypeDef to_fwd;
    to_fwd.RIR = CAN3->sFIFOMailBox[0].RIR | 1; // TXQ
    to_fwd.RDTR = CAN3->sFIFOMailBox[0].RDTR;
    to_fwd.RDLR = CAN3->sFIFOMailBox[0].RDLR;
    to_fwd.RDHR = CAN3->sFIFOMailBox[0].RDHR;

    uint16_t address = CAN3->sFIFOMailBox[0].RIR >> 21;
    
    #ifdef DEBUG_CAN
    puts("CAN3 RX: ");
    puth(address);
    puts("\n");
    #endif

    // CAN data buffer
    uint8_t dat[8];

    switch(address) {
      case GRA_Neu: // ccstalk msg coming into oj533
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN3->sFIFOMailBox[0], i);
        }
        if(dat[0] == volkswagen_pq_compute_checksum(dat, 8)){
          // add permit_braking and recompute the checksum
          dat[1] |=  0b00000001;  // Kodierinfo -> ACC
          dat[2] ^= ~0b00100000;  // Drop first bit of Sender to 0
          dat[2] |=  0b00010000;  //Ensure last bit of Sender is 1
          dat[0] = volkswagen_pq_compute_checksum(dat, 8);
          to_fwd.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
          to_fwd.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
        }
        break;
      default:
        // FWD as-is
        break;
    }
    // send to CAN2 with love from CAN3
    can_send(&to_fwd, 1, false);
    // next
    can_rx(2);
  }
}

void CAN3_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN3);
  llcan_clear_send(CAN3);
}

void TIM3_IRQ_Handler(void) {
  // blink the LED
  if (flash_led >= 10) {
    set_gpio_output(GPIOC, LED_PIN, led_value);
    led_value = !led_value;
    flash_led = 0;
  }
  flash_led++;
  TIM3->SR = 0;
  send = !send;
}

// ***************************** main code *****************************

void gw(void) {
  //read/write
  //maybe implement the ADC and DAC here for pedal like functionality? perhaps VW IMMO replay?
  watchdog_feed();
}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN2_TX_IRQn, CAN2_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN2_RX0_IRQn, CAN2_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN2_SCE_IRQn, CAN2_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN3_TX_IRQn, CAN3_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_RX0_IRQn, CAN3_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_SCE_IRQn, CAN3_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)

  // Should run at around 732Hz (see init below)
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3)

  disable_interrupts();

  // init devices
  clock_init();
  peripherals_init();
  detect_configuration();
  detect_board_type();

  // init microsecond system timer
  // increments 1000000 times per second
  // generate an update to set the prescaler
  TIM2->PSC = 48-1;
  TIM2->CR1 = TIM_CR1_CEN;
  TIM2->EGR = TIM_EGR_UG;
  // use TIM2->CNT to read

  // init board
  current_board->init();
  // enable USB
  #ifdef TGW_USB
    USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
    USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
    usb_init();
  #endif

  // LED business
  set_gpio_mode(GPIOC, LED_PIN, MODE_OUTPUT);
  set_gpio_output_type(GPIOC, LED_PIN, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output(GPIOC, LED_PIN, led_value);

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan1 speed");
  }
  llcan_speed_set = llcan_set_speed(CAN2, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan2 speed");
  }
  llcan_speed_set = llcan_set_speed(CAN3, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan3 speed");
  }

  bool ret = llcan_init(CAN1);
  ret = llcan_init(CAN2);
  ret = llcan_init(CAN3);
  UNUSED(ret);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 7);
  NVIC_EnableIRQ(TIM3_IRQn);
  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  //main GW loop
  while (1) {
    gw();
  }

  return 0;
}
