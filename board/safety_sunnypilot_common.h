#ifndef SAFETY_SUNNYPILOT_COMMON_H
#define SAFETY_SUNNYPILOT_COMMON_H

void mads_acc_main_check(const bool main_on) {
  if (main_on && mads_enabled) {
    controls_allowed = true;
  }
  if (!main_on && acc_main_on_prev) {
    disengageFromBrakes = false;
    controls_allowed = false;
    controls_allowed_long = false;
  }
  acc_main_on_prev = main_on;
}

void mads_lkas_button_check(const bool lkas_pressed) {
  if (lkas_pressed && !lkas_pressed_prev) {
    controls_allowed = true;
  }
  lkas_pressed_prev = lkas_pressed;
}

void mads_exit_controls_check(void) {
  if (alternative_experience & ALT_EXP_MADS_DISABLE_DISENGAGE_LATERAL_ON_BRAKE) {
    disengageFromBrakes = true;
    controls_allowed_long = false;
  } else {
    if ((alternative_experience & ALT_EXP_ENABLE_MADS) && controls_allowed) {
      disengageFromBrakes = true;
    }
    controls_allowed = false;
    controls_allowed_long = false;
  }
}

void mads_resume_controls_check(void) {
  disengageFromBrakes = false;
  if (alternative_experience & ALT_EXP_ENABLE_MADS) {
    controls_allowed = true;
  }
}

typedef struct {
  uint8_t fca_cmd_act;
  uint8_t aeb_cmd_act;
  uint8_t cf_vsm_warn_fca11;
  uint8_t cf_vsm_warn_scc12;
  uint8_t cf_vsm_deccmdact_scc12;
  uint8_t cf_vsm_deccmdact_fca11;
  uint8_t cr_vsm_deccmd_scc12;
  uint8_t cr_vsm_deccmd_fca11;
  uint8_t obj_valid;
  uint8_t acc_objstatus;
  uint8_t acc_obj_lat_pos_1;
  uint8_t acc_obj_lat_pos_2;
  uint8_t acc_obj_dist_1;
  uint8_t acc_obj_dist_2;
  uint8_t acc_obj_rel_spd_1;
  uint8_t acc_obj_rel_spd_2;
} ESCC_Msg;

void send_escc_msg(const ESCC_Msg *msg, int bus_number);

// VW PQ; eEPB
typedef struct {
  uint8_t COUNTER;         // byte 0, start 0, len 4, counter
  uint8_t Verzoegerung;    // byte 3, start 0, len 8, deceleration request (ECD), m/s/s, -7.968 offset, 0.048 scaling
  uint8_t Freigable_Ver;   // byte 4, start 1, len 1, brake enable bit
  uint8_t AutoHold_aktiv;  // byte 4, start 3, len 1, EPB hold active
  uint8_t Bremslicht;      // byte 4, start 7, len 1, brake light
  uint8_t HydrHalten;      // byte 5, start 7, len 1, standstill bit
  uint8_t CHECKSUM;        // byte 7, start 0, len 8, checksum
} mEPB_1;               // EP1, powertrain

typedef struct {
  uint8_t Sta_GRA;         // byte 2, start 6, len 2, ECM cruise state
} mMotor_2;             // MO2

typedef struct {
  uint8_t CHECKSUM;        // byte 0, start 0, len 8, checksum
  uint8_t Verz_EPB_akt;    // byte 1, start 5, len 1
  uint8_t Sta_ACC_Anf;     // byte 4, start 1, len 1
  uint8_t StaBrSyst;       // byte 5, start 7, len 1
} mBremse_8;            // B8

typedef struct {
  uint8_t CHECKSUM;        // byte 0, start 0, len 8, checksum
  uint8_t HydHalten;       // byte 1, start 5, len 1
} mBremse_11;           // B11

typedef struct {
  uint8_t CHECKSUM;        // byte 0, start 0, len 8, checksum
  uint8_t Recall;          // byte 1, start 1, len 1, resume button
} mGRA_Neu;             // GRA

typedef struct {
  uint8_t CHECKSUM;        // byte 0, start 0, len 8, checksum
  uint8_t Sta_ADR;         // byte 1, start 4, len 2, radar cruise state
  uint8_t StSt_Info;       // byte 2, start 0, len 2
  uint8_t FreigSollB;      // byte 2, start 7, len 1, acceleration enable bit
  uint16_t Sollbeschl;     // byte 3, start 0, len 11, acceleration request, m/s/s, -7.22 offset, 0.005 offset
} mACC_System;          // ACS

typedef struct {
  uint8_t CHECKSUM;        // byte 0, start 0, len 8, checksum
  uint8_t Fahrerhinw;      // byte 2, start 0, len 1
  uint8_t Akustik2;        // byte 4, start 2, len 1
} mACC_GRA_Anzeige;     // ACA

typedef struct {
  bool gasPressed;         // 1 if (["Motor_3"]["Fahrpedal_Rohsignal"] / 100.0) > 0 else 0
  bool brakePressed;       // ["Motor_2"]["Bremslichtschalter"]
  bool cruiseCancel;       // ["GRA_Neu"]["GRA_Abbrechen"]
  bool MOB_Standby;        // ["Motor_Bremse"]["MOB_Standby"]
  bool EP1_Freigabe_Ver;   // OEM EPB Verzoegerung release bit
  bool EP1_switchState;    // OEM EPB EP1_Schalterinfo, any non-0-value we consider as eEPB override
  float vEgo;              // ["Bremse_1"]["Geschwindigkeit_neu__Bremse_1_"]
  float aEgo;              // ACS_Sollbeschl
} CarState;             // CS, for misc car signals to be assigned known eEPB variables

typedef struct {
  bool stopped;
  bool stopping;
  bool EPB_brake;
  bool EPB_brake_last;
  bool EPB_enable;
  bool EPB_enable_prev;
  bool EPB_enable_2old;
  bool EPB_active;
  bool ACA_blind;
  uint ACA_blind_counter;
  uint EPB_counter;
  uint accel_diff;
  uint frame;           // 100hz
} ModuleState;          // self, variables for internal module state
                                                                            // bus fwd
void create_mEPB1(const mEPB_1 *msg, int bus_number);                      // 1, 2
void filter_mMotor_2(const mMotor_2 *msg, int bus_number);                  // 0 -> 2
void filter_mBremse_8(const mBremse_8 *msg, int bus_number);                // 0 -> 2
void filter_mBremse_11(const mBremse_11 *msg, int bus_number);              // 0 -> 2
void filter_mGRA_Neu(const mGRA_Neu *msg, int bus_number);                  // 0 -> 2
void filter_mACC_System(const mACC_System *msg, int bus_number);            // 2 -> 0
void filter_mACC_GRA_Anzeige(const mACC_GRA_Anzeige *msg, int bus_number);  // 2 -> 0

#endif
