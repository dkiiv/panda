    // *** Below here is for OEM+ behavior modification of OEM ACC *** //
    // Modify Motor_2, Bremse_8, Bremse_11, GW-EPB_1, ACC_System, ACC_Anzeige, GRA_Neu
    // Create PT-EPB_1

static uint32_t volkswagen_pq_compute_checksum(uint64_t data, int checksum_byte) {
  int len = GET_LEN(data);
  uint8_t checksum = 0U;

  // Simple XOR over the payload, except for the byte where the checksum lives.
  for (int i = 0; i < len; i++) {
    if (i != checksum_byte) {
      checksum ^= (uint8_t)GET_BYTE(data, i);
    }
  }

  return checksum;
}

void filter_motor2(CANPacket_t *to_push, bool active) {
    UNUSED(active);
    to_push->data = (to_push->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}

void filter_bremse8(CANPacket_t *to_push, bool active) {
    UNUSED(active);
    to_push->data = (to_push->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}

void filter_bremse11(CANPacket_t *to_push, bool stopped) {
    UNUSED(stopped);
    to_push->data = (to_push->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}

void filter_epb1(CANPacket_t *to_push, bool stopped) {
    UNUSED(stopped);
    to_push->data = (to_push->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}

void filter_ACC_System(CANPacket_t *to_push, bool epb_freigabe) {
    UNUSED(epb_freigabe);
    to_push->data = (to_push->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}

void filter_ACC_Anzeige(CANPacket_t *to_push, bool blind) {
    UNUSED(blind);
    to_push->data = (to_push->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}

void filter_GRA_Neu(CANPacket_t *to_push, bool resume) {
    UNUSED(resume);
    to_push->data = (to_push->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}

void create_epb_control(CANPacket_t *to_push, double apply_brake, bool EPB_enabled) {
    UNUSED(apply_brake);
    UNUSED(EPB_enabled);
    to_push->data = (to_push->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}