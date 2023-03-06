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

#define MAX_TIMEOUT 50U
uint32_t timeout = 0;
uint32_t timeout_f10 = 0;
uint32_t timeout_f11 = 0;

#define NO_FAULT 0U
#define FAULT_BAD_CHECKSUM 1U
#define FAULT_SEND 2U
#define FAULT_SCE 3U
#define FAULT_STARTUP 4U
#define FAULT_TIMEOUT 5U
#define FAULT_INVALID 6U
#define STATE_AEB_CTRL 7U

uint8_t state = FAULT_STARTUP;
uint8_t ctrl_mode = 0;
bool send = 0;

// Bus 0: Ext Can
// Bus 1: Car PTCan
// Bus 2: GW PTCan

//------------- BUS 0 - EXT CAN --------------//

uint8_t  msgPump = 0;
uint8_t engagementCounter = 0;
// mACC_System
uint8_t  ACS_Zaehler = 0;          //counter
uint8_t  ACS_Sta_ADR = 2;          //ADR Status (2 inactive)
uint8_t  ACS_FreigSollB = 0;       //Activation of ACS_Sollbeschl (1 allowed)
uint8_t  ACS_StSt_Info = 1;        //StartStopRequest (1 Engine start not needed) | this may be subject to change in vehicles which utilize start stop
uint8_t  ACS_MomEingriff = 0;      //Torque intervention (Prevent whiplash?) (0 Allow whiplash)
uint8_t  ACS_Typ_ACC = 0;          //ADR Type (0 normal ACC | 1 ACC Follow2Stop) | this may be subject to change as not all vehicles will support FtS ACC
uint16_t ACS_Sollbeschl = 2046;    //Acceleration Request (2046(10.23) ADR Inactive)
uint8_t  ACS_Anhaltewunsch = 0;    //Stopping request (0 no stop request)
uint8_t  ACS_zul_Regelabw = 254;   //Allowed request deviation (254 ADR not active) | Ties into ACS_Sollbeschl problem
uint8_t  ACS_max_AendGrad = 0;     //Allowed gradient changes (0) | sg is unknown, will change later

// mACC_GRA_Anziege
uint8_t  ACA_StaACC = 1;           //ADR Status in cluster (1 ACC ok but disabled)
uint8_t  ACA_AnzDisplay = 0;       //ADR Display Status (0 no-Display)
uint8_t  ACA_Zeitluecke = 7;       //Display set time gap (0 not defined / 1-15 Distances)
uint8_t  ACA_V_Wunsch = 255;       //Display set speed, eventually tie this into displaying the set cruisecontrol speed without OP (255 not set yet)
uint8_t  ACA_Aend_Zeitluecke = 1;  //Display started
uint8_t  ACA_PrioDisp = 1;         //ACC Display priority (0 High Prio / 1 Prio / 2 Low Prio / 3 No Request)
uint8_t  ACA_gemZeitl = 0;         //Average follow distance (0 No lead / 1-15 Actual average distance)

//------------- BUS 1 - CAR PTCAN ------------//

#define mMotor_2 0x288
#define mMotor_3 0x380
#define mBremse_3 0x4A0
#define mACC_GRA_Anziege 0x56a
  //Brake Pressed
bool MO2_BTS = 0;
  //Wheel speed sensors
uint16_t BR3_Rad_kmh_VL = 0;
uint16_t BR3_Rad_kmh_VR = 0;
uint16_t BR3_Rad_kmh_HL = 0;
uint16_t BR3_Rad_kmh_HR = 0;
float kphMphConv = 0.621371;
uint16_t vEgoKPH = 0;
uint16_t vEgoMPH = 0;
  //Pedal position
uint8_t MO3_Pedalwert = 0;

//------------- BUS 2 - GW PTCAN -------------//

#define GRA_Neu 0x38A
  //Stalk button status 
uint8_t GRA_Lever_Pos = 0;  //GRA_Hauptschalt and GRA_Abbrechen
uint8_t GRA_Tip_Pos = 0;    //GRA_Tip_Down and GRA_Tip_Up

//------------- CAN FWDing below -------------//

void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {

    CAN_FIFOMailBox_TypeDef to_fwd;
    to_fwd.RIR = CAN1->sFIFOMailBox[0].RIR | 1; // TXQ
    to_fwd.RDTR = CAN1->sFIFOMailBox[0].RDTR;
    to_fwd.RDLR = CAN1->sFIFOMailBox[0].RDLR;
    to_fwd.RDHR = CAN1->sFIFOMailBox[0].RDHR;

    uint16_t address = CAN1->sFIFOMailBox[0].RIR >> 21;

    #ifdef DEBUG_CAN
    puts("CAN1 RX: ");
    puth(address);
    puts("\n");
    #endif

    // CAN data buffer
    uint8_t dat[8];

    switch (address) {
      case CAN_UPDATE:
        if (GET_BYTES_04(&CAN1->sFIFOMailBox[0]) == 0xdeadface) {
          if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x0ab00b1e) {
            enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
            NVIC_SystemReset();
          } else if (GET_BYTES_48(&CAN1->sFIFOMailBox[0]) == 0x02b00b1e) {
            enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
            NVIC_SystemReset();
          } else {
            puts("Failed entering Softloader or Bootloader\n");
          }
        }
        break;
      case mACC_GRA_Anziege:
        msgPump = 200;                  //sets msgPump buffer to 2 seconds. radar message will shutoff after 2 seconds of no ign
        if (GRA_Tip_Pos >= 1) {
          if (ACA_V_Wunsch == 255 || (ACS_Sta_ADR >= 2 && GRA_Tip_Pos == 2)) {
            vEgoKPH = ((BR3_Rad_kmh_VL + BR3_Rad_kmh_VR + BR3_Rad_kmh_HL + BR3_Rad_kmh_HR) / 4) & 0xFFFFFFFFFFFFFFF;
            vEgoMPH = (vEgoKPH * kphMphConv) * 0.01;
            ACA_V_Wunsch = ((int)((vEgoMPH + 2) / 5)) * 5;
          } else if (GRA_Tip_Pos == 2) {
            ACA_V_Wunsch = ACA_V_Wunsch - 5;
          } else if (GRA_Tip_Pos == 1 && ACS_Sta_ADR >= 2) {
            ACA_V_Wunsch = ACA_V_Wunsch;
          } else {
            ACA_V_Wunsch = ACA_V_Wunsch + 5;
          }
          engagementCounter++;
          ACS_Sta_ADR = 1;              //ADR Status (1 active)
          ACS_FreigSollB = 1;           //Activation of ACS_Sollbeschl (1 allowed)
          ACA_StaACC = 3;               //ADR Status in cluster (3 ACC Active)
        }
        if (GRA_Lever_Pos >= 1 || MO2_BTS) {  //This turns off ACC control
          if (GRA_Lever_Pos == 1) {  //Resets the setpoint speed when 3 position switch is flicked into toggle off
            ACA_StaACC = 1;             //ADR Status in cluster (1 ACC ok but disabled)
          }
          ACS_Sta_ADR = 2;              //ADR Status (2 passive)
          ACS_FreigSollB = 0;           //Activation of ACS_Sollbeschl (0 not allowed)
          ACA_StaACC = 2;               //ADR Status in cluster (2 ACC Passive)
        }
        if (engagementCounter >= 1) {
          ACA_AnzDisplay = 1;           //ADR Display Status (1 Display)
          engagementCounter &= 100;     //Constrain engagement counter to 1 second
        } else {
          ACA_AnzDisplay = 0;           //ADR Display Status (0 no display)
        }
        if (MO3_Pedalwert > 0 && ACA_StaACC == 3) {   // This sets ACA_StaACC to 4, ACC in background as driver is overriding it
          ACA_StaACC = 4;
        }
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        if(dat[0] == volkswagen_pq_compute_checksum(dat, 8)){
          dat[0] = volkswagen_pq_compute_checksum(dat, 8);
          dat[1] |= ACA_StaACC << 5U;
          dat[2] |= ACA_AnzDisplay << 6U | ACA_Zeitluecke << 2U;
          dat[3] = ACA_V_Wunsch;
          dat[4] |= ACA_PrioDisp << 3U;
          dat[5] |= ACA_gemZeitl << 4U;
          dat[7] |= ACA_Aend_Zeitluecke << 5U;
          to_fwd.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
          to_fwd.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
        }        
        break;
      default:
        // FWD as-is
        break;
    }
    // no forward, can 1 is injection
    can_send(&to_fwd, 0, false);
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

    CAN_FIFOMailBox_TypeDef to_fwd;
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
      case mMotor_2:  // msg containing brake pressed data
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN2->sFIFOMailBox[0], i);
        }
        MO2_BTS = (dat[2] && 0b01000000) >> 6U;
        break;
      case mMotor_3:  // msg containing pedal value
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN2->sFIFOMailBox[0], i);
        }
        MO3_Pedalwert = (int)((dat[2] * 0.04) / 100);
        break;
      case mBremse_3: // msg containing wheel speed data
        for (int i=0; i<8; i++) {
          dat[i] = GET_BYTE(&CAN2->sFIFOMailBox[0], i);
        }
        BR3_Rad_kmh_VL = ((dat[0] >> 1) | dat[1]);
        BR3_Rad_kmh_VR = ((dat[2] >> 1) | dat[3]);
        BR3_Rad_kmh_HL = ((dat[4] >> 1) | dat[5]);
        BR3_Rad_kmh_HR = ((dat[6] >> 1) | dat[7]);
        break;
      default:
        // FWD as-is
        break;
    }
    // send to CAN3 with love from CAN2
    can_send(&to_fwd, 2, false);
    // next
    can_rx(1);
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
          GRA_Lever_Pos = (dat[1] >> 6U) & 0x2;
          GRA_Tip_Pos = (dat[3] >> 6U) & 0x2;
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
    can_rx(3);
  }
}

void CAN3_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN3);
  llcan_clear_send(CAN3);
}

void TIM3_IRQ_Handler(void) {
  // inject messages onto ext can into gateway/OP relay
  //100hz
  if (true) {
    msgPump--;            // bleeds off msgPump after 2 seconds
    if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
      uint8_t dat[8]; //SEND mACC_System 0x368

      dat[0] = volkswagen_pq_compute_checksum(dat, 8);
      dat[1] = ACS_Zaehler << 4U | ACS_Sta_ADR << 2U;
      dat[2] = ACS_StSt_Info << 6U | ACS_MomEingriff << 5U | ACS_Typ_ACC << 3U | ACS_FreigSollB;
      dat[3] = (ACS_Sollbeschl >> 3U) & 0xFF;
      dat[4] = ((ACS_Sollbeschl << 8U) & 7U) << 5U | ACS_Anhaltewunsch << 1U;
      dat[5] = ACS_zul_Regelabw;
      dat[6] = ACS_max_AendGrad;
      dat[7] = 0;

      CAN_FIFOMailBox_TypeDef to_send;
      to_send.RDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
      to_send.RDHR = dat[4] | (dat[5] << 8) | (dat[6] << 16) | (dat[7] << 24);
      to_send.RDTR = 8;
      to_send.RIR = (0x368 << 21) | 1U;
      can_send(&to_send, 0, false);;

      ACS_Zaehler++;
      ACS_Zaehler &= 15;
    }
    else {
      // old can packet hasn't sent!
      #ifdef DEBUG_CAN
        puts("CAN1 MISS1\n");
      #endif
    }
  }
}

// ***************************** main code *****************************


void gw(void) {
  // read/write
  // maybe we can implement the ADC and DAC here for pedal functionality?
  watchdog_feed();
}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  // REGISTER_INTERRUPT(CAN2_TX_IRQn, CAN2_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  // REGISTER_INTERRUPT(CAN2_RX0_IRQn, CAN2_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  // REGISTER_INTERRUPT(CAN2_SCE_IRQn, CAN2_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
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

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan1 speed");
  }
  // llcan_speed_set = llcan_set_speed(CAN2, 5000, false, false);
  // if (!llcan_speed_set) {
  //   puts("Failed to set llcan2 speed");
  // }
  llcan_speed_set = llcan_set_speed(CAN3, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan3 speed");
  }

  bool ret = llcan_init(CAN1);
  // ret = llcan_init(CAN2);
  ret = llcan_init(CAN3);
  UNUSED(ret);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 7);
  NVIC_EnableIRQ(TIM3_IRQn);

  // TODO: figure out a function for these GPIOs
  // set_gpio_mode(GPIOB, 12, MODE_OUTPUT);
  // set_gpio_output_type(GPIOB, 12, OUTPUT_TYPE_PUSH_PULL);
  // set_gpio_output(GPIOB, 12, 1);

  // set_gpio_mode(GPIOB, 13, MODE_OUTPUT);
  // set_gpio_output_type(GPIOB, 13, OUTPUT_TYPE_PUSH_PULL);

  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main pedal loop
  while (1) {
    gw();
  }

  return 0;
}
