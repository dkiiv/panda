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

void parse_ACC_System_state(ACC_State *ACS) {
    CANPacket_t to_push;

    ACS->Sta_ADR = 0x00;
    ACS->Sollbeschl = 0x00;
};

void filter_motor2(EPB_State *state) {
    CANPacket_t to_push;

    if (state->EPB_active) {
        to_push.data = (to_push.data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
    };
}

void filter_bremse8(EPB_State *state) {
    CANPacket_t to_push;

    if (state->EPB_active) {
        to_push.data = (to_push.data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
    };
}

void filter_bremse11(Device_State *device) {
    CANPacket_t to_push;

    if (device->stopped) {
        to_push.data = (to_push.data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
    };
}

void filter_epb1(Device_State *device) {
    CANPacket_t to_push;

    if (device->stopped) {
        to_push.data = (to_push.data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
    };
}

void filter_ACC_System(EPB_State *state) {
    CANPacket_t to_push;

    if (state->EPB_active) {
        to_push.data = (to_push.data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
    };
}

void filter_ACC_Anzeige(EPB_State *state) {
    CANPacket_t to_push;

    if (state->ACC_anz_blind) {
        to_push.data = (to_push.data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
    };
}

void filter_GRA_Neu(Device_State *device) {
    CANPacket_t to_push;

    if (device->resume) {
        to_push.data = (to_push.data & 0xFFFFFF00) | 0xAB; // Example: Update lowest byte
    };
}

void create_epb_control(EPB_msg *msg, EPB_State *epb) {
    msg->EP1_Zaehler = epb->Zaehler;
    msg->EP1_Verzoegerung = epb->apply_brake;
    msg->EP1_Freigabe_Ver = epb->EPB_enabled ? 1 | 0;
    msg->EP1_Bremslicht = epb->apply_brake != 0 ? 1 | 0;
    msg->EP1_HydrHalten = epb->EPB_enabled ? 1 | 0;
    msg->EP1_AutoHold_aktiv = 1;
    msg->EP1_Checksum = volkswagen_pq_compute_checksum(msg, 7);  // TODO: verify / fix that this creates proper checksum
    send_epb_msg(&msg, 1);
}