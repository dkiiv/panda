// /////////// //
// White Panda //
// /////////// //

void white_enable_can_transceiver(uint8_t transceiver, bool enabled) {
  switch (transceiver){
    case 1U:
      set_gpio_output(GPIOC, 1, !enabled);
      break;
    case 2U:
      set_gpio_output(GPIOC, 13, !enabled);
      break;
    case 3U:
      set_gpio_output(GPIOA, 0, !enabled);
      break;
    default:
      puts("Invalid CAN transceiver ("); puth(transceiver); puts("): enabling failed\n");
      break;
  }
}

void white_enable_can_transceivers(bool enabled) {
  uint8_t t1 = enabled ? 1U : 2U;  // leave transceiver 1 enabled to detect CAN ignition
  for(uint8_t i=t1; i<=3U; i++) {
    white_enable_can_transceiver(i, enabled);
  }
}

void white_set_led(uint8_t color, bool enabled) {
  switch (color){
    case LED_RED:
      set_gpio_output(GPIOC, 9, !enabled);
      break;
     case LED_GREEN:
      set_gpio_output(GPIOC, 7, !enabled);
      break;
    case LED_BLUE:
      set_gpio_output(GPIOC, 6, !enabled);
      break;
    default:
      break;
  }
}

void white_set_usb_power_mode(uint8_t mode){
  UNUSED(mode);
}

void white_set_gps_mode(uint8_t mode) {
  UNUSED(mode);
}

void white_set_can_mode(uint8_t mode){
  switch (mode) {
    case CAN_MODE_NORMAL:
      // B8,B9: normal CAN1 mode
      set_gpio_alternate(GPIOB, 8, GPIO_AF8_CAN1);
      set_gpio_alternate(GPIOB, 9, GPIO_AF8_CAN1);

      // B5,B6: normal CAN2 mode
      set_gpio_alternate(GPIOB, 5, GPIO_AF9_CAN2);
      set_gpio_alternate(GPIOB, 6, GPIO_AF9_CAN2);

      // A8,A15: normal CAN3 mode
      set_gpio_alternate(GPIOA, 8, GPIO_AF11_CAN3);
      set_gpio_alternate(GPIOA, 15, GPIO_AF11_CAN3);
      break;
    default:
      puts("Tried to set unsupported CAN mode: "); puth(mode); puts("\n");
      break;
  }
}

uint32_t white_read_current(void){
  return 0U;
}

void white_usb_power_mode_tick(uint32_t uptime){
  UNUSED(uptime);
}

void white_set_ir_power(uint8_t percentage){
  UNUSED(percentage);
}

void white_set_fan_power(uint8_t percentage){
  UNUSED(percentage);
}

bool white_check_ignition(void){
  return 0U;
}

void white_set_phone_power(bool enabled){
  UNUSED(enabled);
}

void white_set_clock_source_mode(uint8_t mode){
  UNUSED(mode);
}

void white_set_siren(bool enabled){
  UNUSED(enabled);
}

void white_grey_common_init(void) {
  common_init_gpio();

  // A2, A3: USART 2 for debugging
  set_gpio_alternate(GPIOA, 2, GPIO_AF7_USART2);
  set_gpio_alternate(GPIOA, 3, GPIO_AF7_USART2);

  // A4, A5, A6, A7: SPI
  set_gpio_alternate(GPIOA, 4, GPIO_AF5_SPI1);
  set_gpio_alternate(GPIOA, 5, GPIO_AF5_SPI1);
  set_gpio_alternate(GPIOA, 6, GPIO_AF5_SPI1);
  set_gpio_alternate(GPIOA, 7, GPIO_AF5_SPI1);

  // C10, C11: L-Line setup (USART3)
  set_gpio_alternate(GPIOC, 10, GPIO_AF7_USART3);
  set_gpio_alternate(GPIOC, 11, GPIO_AF7_USART3);
  set_gpio_pullup(GPIOC, 11, PULL_UP);

  // Enable CAN transceivers
  white_enable_can_transceivers(true);

  // Disable LEDs
  white_set_led(LED_RED, false);
  white_set_led(LED_GREEN, false);
  white_set_led(LED_BLUE, false);

  // Set normal CAN mode
  white_set_can_mode(CAN_MODE_NORMAL);
}

void white_init(void) {
  white_grey_common_init();

  // Set ESP off by default
  current_board->set_gps_mode(GPS_DISABLED);
}

const harness_configuration white_harness_config = {
  .has_harness = false
};

const board board_white = {
  .board_type = "White",
  .harness_config = &white_harness_config,
  .init = white_init,
  .enable_can_transceiver = white_enable_can_transceiver,
  .enable_can_transceivers = white_enable_can_transceivers,
  .set_led = white_set_led,
  .set_usb_power_mode = white_set_usb_power_mode,
  .set_gps_mode = white_set_gps_mode,
  .set_can_mode = white_set_can_mode,
  .usb_power_mode_tick = white_usb_power_mode_tick,
  .check_ignition = white_check_ignition,
  .read_current = white_read_current,
  .set_fan_power = white_set_fan_power,
  .set_ir_power = white_set_ir_power,
  .set_phone_power = white_set_phone_power,
  .set_clock_source_mode = white_set_clock_source_mode,
  .set_siren = white_set_siren
};
