    // *** Below here is for OEM+ behavior modification of OEM ACC *** //
    // Modify Motor_2, Bremse_8, Bremse_11, GW-EPB_1, ACC_System, ACC_Anzeige, GRA_Neu
    // Create PT-EPB_1

static uint32_t volkswagen_pq_compute_checksum(const CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);
  uint8_t checksum = 0U;
  int checksum_byte = (addr == MSG_MOTOR_5) ? 7 : 0;

  // Simple XOR over the payload, except for the byte where the checksum lives.
  for (int i = 0; i < len; i++) {
    if (i != checksum_byte) {
      checksum ^= (uint8_t)GET_BYTE(to_push, i);
    }
  }

  return checksum;
}

void filter_motor2(CANPacket_t *packet, bool active) {
    UNUSED(active);
    packet->data = (packet->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}

void filter_bremse8(CANPacket_t *packet, bool active) {
    UNUSED(active);
    packet->data = (packet->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}

void filter_bremse11(CANPacket_t *packet, bool stopped) {
    UNUSED(stopped);
    packet->data = (packet->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}

void filter_epb1(CANPacket_t *packet, bool stopped) {
    UNUSED(stopped);
    packet->data = (packet->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}

void filter_ACC_System(CANPacket_t *packet, bool epb_freigabe) {
    UNUSED(epb_freigabe);
    packet->data = (packet->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}

void filter_ACC_Anzeige(CANPacket_t *packet, bool blind) {
    UNUSED(blind);
    packet->data = (packet->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}

void filter_GRA_Neu(CANPacket_t *packet, bool stopped) {
    UNUSED(stopped);
    packet->data = (packet->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}

void create_epb_control(CANPacket_t *packet, double apply_brake, bool EPB_enabled) {
    UNUSED(apply_brake);
    UNUSED(EPB_enabled);
    packet->data = (packet->data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
}