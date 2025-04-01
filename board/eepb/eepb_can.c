/*
                                                                            // bus fwd
void create_mEPB1(const ptEPB_1 *msg, int bus_number);                      // 1, 2
void filter_mMotor_2(const mMotor_2 *msg, int bus_number);                  // 0 -> 2
void filter_mBremse_8(const mBremse_8 *msg, int bus_number);                // 0 -> 2
void filter_mBremse_11(const mBremse_11 *msg, int bus_number);              // 0 -> 2
void filter_mGRA_Neu(const mGRA_Neu *msg, int bus_number);                  // 0 -> 2
void filter_mACC_System(const mACC_System *msg, int bus_number);            // 2 -> 0
void filter_mACC_GRA_Anzeige(const mACC_GRA_Anzeige *msg, int bus_number);  // 2 -> 0
*/

#include <math.h>
#include "board/safety/safety_volkswagen_pq_eepb.h"

double limit_jerk(double accel, double prev_accel, double max_jerk, double dt) {
    double max_delta_accel = max_jerk * dt;
    double delta_accel = fmax(-max_delta_accel, fmin(accel - prev_accel, max_delta_accel));
    return prev_accel + delta_accel;
}

void EPB_handler(const CarState *CS, ModuleState *self) {
    if ((CS->aEgo < 0) && ((CS->MOB_Standby && CS->vEgo <= 18) || self->EPB_enable)) {
        if (!self->EPB_enable) {
            self->EPB_counter = 0;
            self->EPB_brake = 0;
            self->EPB_enable = 1;
            self->EPB_brake_last = CS->aEgo;
        } else {
            self->EPB_brake = self->stopping ? limit_jerk(-4, self->EPB_brake_last, 0.7, 0.02) : CS->aEgo;
            self->EPB_brake_last = self->EPB_brake;
        }
        self->EPB_counter++;
    } else {
        if (self->EPB_enable && self->EPB_counter < 10) {
            self->EPB_counter++;
        } else {
            self->EPB_brake = 0;
            self->EPB_enable = 0;
        }
    }

    if (CS->gasPressed || CS->brakePressed || CS->cruiseCancel || (CS->EP1_Freigabe_Ver || CS->EP1_switchState)) {
        if (self->EPB_enable) {
            self->ACA_blind = 1;
        }
        self->EPB_brake = 0;
        self->EPB_enable = 0;
        self->EPB_enable_prev = 0;
        self->EPB_enable_2old = 0;
    }

    if (self->ACA_blind && self->ACA_blind_counter < 150) {
        self->ACA_blind_counter++;
    } else {
        self->ACA_blind = 0;
        self->ACA_blind_counter = 0;
    }

    self->EPB_active = ((self->EPB_enable_2old && !self->EPB_enable) || self->EPB_enable);
    self->EPB_enable_2old = self->EPB_enable_prev;
    self->EPB_enable_prev = self->EPB_enable;
}

void create_mEPB1(const mEPB_1 *msg, const CarState *CS, const ModuleState *self, int bus_number) {
    uint8_t dat[8];
    if (!CS->EP1_Freigabe_Ver && !CS->EP1_switchState) {
        dat[0] = (msg->COUNTER << 4);
        if (bus_number == 1) {
          // to powertrain
            dat[1] = 0;
            dat[2] = 0;
            //         Verzoegerung
            dat[3] = ((self->EPB_brake + 7.968) / 0.048);
            //         Freigable_Ver             AutoHold_aktiv            Bremslicht
            dat[4] = (self->EPB_enable << 6) | (self->EPB_enable << 4) | (self->EPB_brake != 0);
            //         HydrHalten
            dat[5] = (self->EPB_enable);
            dat[6] = 0;
        } else {
          // to radar
            dat[1] = 0;
            dat[2] = 0;
            dat[3] = 0;
            //        AutoHold_aktiv
            dat[4] = (1 << 4);
            //        HydrHalten
            dat[5] = (self->stopped);
            dat[6] = 0;
        }
    } else {
        dat[0] = (msg->COUNTER << 4) | (msg->OEM[0] & 0b1111);
        dat[1] = msg->OEM[1];
        dat[2] = msg->OEM[2];
        dat[3] = msg->OEM[3];
        dat[4] = msg->OEM[4];
        dat[5] = msg->OEM[5];
        dat[6] = msg->OEM[6];
    }
    //        checksum
    dat[7] = dat[0] ^ dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5] ^ dat[6];

    CANPacket_t to_send;
    to_send.extended = 1;
    to_send.addr = EPB_1;
    to_send.bus = bus_number;
    to_send.data_len_code = sizeof(dat);
    memcpy(to_send.data, dat, sizeof(dat));
    
    can_set_checksum(&to_send);
    can_send(&to_send, bus_number, true);
    }

void filter_mMotor_2(const mMotor_2 *msg, const ModuleState *self, int bus_number) {
    uint8_t dat[8];
    dat[0] = msg->msg[0];
    dat[1] = msg->msg[1];
    dat[2] = (self->EPB_active ? (msg->msg[2] & 0b01111101) : msg->msg[2]);
    dat[3] = msg->msg[3];
    dat[4] = msg->msg[4];
    dat[5] = msg->msg[5];
    dat[6] = msg->msg[6];
    dat[7] = msg->msg[7];

    CANPacket_t to_send;
    to_send.extended = 1;
    to_send.addr = MOTOR_2;
    to_send.bus = bus_number;
    to_send.data_len_code = sizeof(dat);
    memcpy(to_send.data, dat, sizeof(dat));
    
    can_set_checksum(&to_send);
    can_send(&to_send, bus_number, true);
    }