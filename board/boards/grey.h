// ////////// //
// Grey Panda //
// ////////// //

// Most hardware functionality is similar to white panda

void grey_init(void) {
  white_grey_common_init();
}

void grey_set_gps_mode(uint8_t mode) {
  UNUSED(mode);
}

const board board_grey = {
  .board_type = "Grey",
  .harness_config = &white_harness_config,
  .init = grey_init,
  .enable_can_transceiver = white_enable_can_transceiver,
  .enable_can_transceivers = white_enable_can_transceivers,
  .set_led = white_set_led,
  .set_usb_power_mode = white_set_usb_power_mode,
  .set_gps_mode = grey_set_gps_mode,
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
