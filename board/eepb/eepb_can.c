void send_escc_msg(const ESCC_Msg *msg, const int bus_number) {
      uint8_t dat[8];
      dat[0] = (msg->fca_cmd_act) | (msg->cf_vsm_warn_fca11 << 1) | (msg->aeb_cmd_act << 3) | 
               (msg->cf_vsm_warn_scc12 << 4) | (msg->cf_vsm_deccmdact_scc12 << 6) | (msg->cf_vsm_deccmdact_fca11 << 7);
      dat[1] = (msg->cr_vsm_deccmd_scc12);
      dat[2] = (msg->obj_valid) | (msg->acc_objstatus << 1);
      dat[3] = (msg->acc_obj_lat_pos_1);
      dat[4] = (msg->acc_obj_lat_pos_2) | (msg->acc_obj_dist_1 << 1);
      dat[5] = (msg->acc_obj_dist_2) | (msg->acc_obj_rel_spd_1 << 4);
      dat[6] = (msg->acc_obj_rel_spd_2);
      dat[7] = (msg->cr_vsm_deccmd_fca11);
    
      CANPacket_t to_send;
      to_send.extended = CAN_ESCC_OUTPUT >= 0x800 ? 1 : 0;
      to_send.addr = CAN_ESCC_OUTPUT;
      to_send.bus = bus_number;
      to_send.data_len_code = sizeof(dat);
      memcpy(to_send.data, dat, sizeof(dat));
      
      can_set_checksum(&to_send);
      can_send(&to_send, bus_number, true);
    }

/*
        example ^^
                                                                            // bus fwd
void create_mEPB1(const ptEPB_1 *msg, int bus_number);                      // 1, 2
void filter_mMotor_2(const mMotor_2 *msg, int bus_number);                  // 0 -> 2
void filter_mBremse_8(const mBremse_8 *msg, int bus_number);                // 0 -> 2
void filter_mBremse_11(const mBremse_11 *msg, int bus_number);              // 0 -> 2
void filter_mGRA_Neu(const mGRA_Neu *msg, int bus_number);                  // 0 -> 2
void filter_mACC_System(const mACC_System *msg, int bus_number);            // 2 -> 0
void filter_mACC_GRA_Anzeige(const mACC_GRA_Anzeige *msg, int bus_number);  // 2 -> 0
*/


void create_mEPB1(const mEPB_1 *msg, int bus_number) {
/*
    typedef struct {
    uint COUNTER;         // byte 0, start 0, len 4, counter
    uint Verzoegerung;    // byte 3, start 0, len 8, deceleration request (ECD), m/s/s, -7.968 offset, 0.048 scaling
    uint Freigable_Ver;   // byte 4, start 1, len 1, brake enable bit
    uint AutoHold_aktiv;  // byte 4, start 3, len 1, EPB hold active
    uint Bremslicht;      // byte 4, start 7, len 1, brake light
    uint HydrHalten;      // byte 5, start 7, len 1, standstill bit
    uint CHECKSUM;        // byte 7, start 0, len 8, checksum
    } mEPB_1;              // EP1, powertrain
*/
    uint8_t dat[8];
    dat[0];
    if (bus_number == 1) {
        dat[1];
        dat[2];
        dat[3];
        dat[4];
        dat[5];
        dat[6];
    } else {
        dat[1];
        dat[2];
        dat[3];
        dat[4];
        dat[5];
        dat[6];
    }
    dat[7] = dat[0] ^ dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5] ^ dat[6];

    CANPacket_t to_send;
    to_send.extended = 1;
    to_send.addr = 0x5C0;
    to_send.bus = bus_number;
    to_send.data_len_code = sizeof(dat);
    memcpy(to_send.data, dat, sizeof(dat));
    
    can_set_checksum(&to_send);
    can_send(&to_send, bus_number, true);
    }