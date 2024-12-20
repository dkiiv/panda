#pragma once

#include "safety_declarations.h"
#include "safety/pqcan.h"

//  MSG's filtered
#define MSG_ACC_SYSTEM     0x368  // 2 -> 0
#define MSG_ACC_ANZEIGE    0x56A  // 2 -> 0
#define MSG_MOTOR_2        0x288  // 0 -> 2
#define MSG_BREMSE_8       0x1AC  // 0 -> 2
#define MSG_BREMSE_11      0x5B7  // 0 -> 2
#define MSG_GRA_NEU        0x38A  // 0 -> 2
#define MSG_EPB_1          0x5C0  // * -> 1, 0 -> 2

//  MSG's for car state
#define MSG_MOTOR_3        0x380  // gas pedal
#define MSG_BREMSE_1       0x1A0  // vehicle speed
#define MSG_BREMSE_5       0x4A8  // brake pedal

bool stopping = 0;
bool stopped = 0;
bool resume = 0;
int frame = 0;
bool ACS_Anhaltewunsch;
int ACS_Sta_ADR;
double vEgo;

typedef struct {
    bool EPB_enable;
    double EPB_brake;
    bool EPB_active;
} EPB_Handler_Result;

typedef struct {
    bool EPB_enable;
    bool EPB_enable_prev;
    bool EPB_enable_2old;
    bool ACC_anz_blind;
    int EPB_counter;
    int ACC_anz_blind_counter;
    double EPB_brake;
    double EPB_brake_last;
} EPB_State;

typedef struct {
    bool gasPressed;
    bool brakePressed;
} CarState;

double limit_jerk(double accel, double prev_accel, double max_jerk, double dt) {
    double max_delta_accel = max_jerk * dt;
    double delta_accel = (accel - prev_accel) > max_delta_accel ? max_delta_accel :
                         (accel - prev_accel) < -max_delta_accel ? -max_delta_accel :
                         (accel - prev_accel);
    return prev_accel + delta_accel;
}

EPB_Handler_Result EPB_handler(CarState CS, int ACS_Sta_ADR, double ACS_Sollbeschl, double vEgo, bool stopping, EPB_State *state) {
    if (ACS_Sta_ADR == 1 && ACS_Sollbeschl < 0 && vEgo <= (18 * KPH_TO_MS)) {
            // First frame of EPB entry
        if (!state->EPB_enable) {
            state->EPB_counter = 0;
            state->EPB_brake = 0;
            state->EPB_enable = true;
            state->EPB_brake_last = ACS_Sollbeschl;
        } else {
            state->EPB_brake = stopping ? limit_jerk(-4.0, state->EPB_brake_last, 0.7, 0.02) : ACS_Sollbeschl;
            state->EPB_brake_last = state->EPB_brake;
        }
        state->EPB_counter++;
    } else {
        if (state->EPB_enable && state->EPB_counter < 10) {
            state->EPB_counter++;
        } else {
            state->EPB_brake = 0;
            state->EPB_enable = false;
        }
    }

    if (CS.gasPressed || CS.brakePressed) {
        if (state->EPB_enable) {
            state->ACC_anz_blind = true;
        }
        state->EPB_brake = 0;
        state->EPB_enable = false;
        state->EPB_enable_prev = false;
        state->EPB_enable_2old = false;
    }

    if (state->ACC_anz_blind && state->ACC_anz_blind_counter < 150) {
        state->ACC_anz_blind_counter++;
    } else {
        state->ACC_anz_blind = false;
        state->ACC_anz_blind_counter = 0;
    }

    // Update EPB historical states and calculate EPB_active
    state->EPB_active = (state->EPB_enable_2old && !state->EPB_enable) || state->EPB_enable;
    state->EPB_enable_2old = state->EPB_enable_prev;
    state->EPB_enable_prev = state->EPB_enable;

    EPB_Handler_Result result = {
        .EPB_enable = state->EPB_enable,
        .EPB_brake = state->EPB_brake,
        .EPB_active = state->EPB_active,
    };
    return result;
}

void default_rx_hook(const CANPacket_t *to_push) {
  UNUSED(to_push);
}

// *** no output safety mode ***

static safety_config nooutput_init(uint16_t param) {
  UNUSED(param);
  return (safety_config){NULL, 0, NULL, 0};
}

static bool nooutput_tx_hook(const CANPacket_t *to_send) {
  UNUSED(to_send);
  return true;
}

static int default_fwd_hook(CANPacket_t *to_push) {
  const int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);
  int bus_fwd = -1;

  switch (bus_num) {
    case 0:
      if (addr == MSG_MOTOR_2) filter_motor2(to_push, EPB_Handler_Result.EPB_active);
      if (addr == MSG_BREMSE_8) filter_bremse8(to_push, EPB_Handler_Result.EPB_active);
      if (addr == MSG_BREMSE_11) filter_bremse11(to_push, stopped);
      if (addr == MSG_EPB_1) filter_epb1(to_push, stopped);
      if (addr == MSG_GRA_NEU) {
        resume = stopped && (frame % 100 < 50);
        filter_GRA_Neu(to_push, resume);
      };
      bus_fwd = 2;
      break;
    case 2:
      if (addr == MSG_ACC_SYSTEM) filter_ACC_System(to_push, EPB_Handler_Result.EPB_active);
      if (addr == MSG_ACC_ANZEIGE) filter_ACC_Anzeige(to_push, EPB_State.ACC_anz_blind);
      bus_fwd = 0;
      break;
    default:
      bus_fwd = -1;
      break;
  }
  return bus_fwd;
}

const safety_hooks nooutput_hooks = {
  .init = nooutput_init,
  .rx = default_rx_hook,
  .tx = nooutput_tx_hook,
  .fwd = default_fwd_hook,
};

// *** all output safety mode ***

// Enables passthrough mode where relay is open and bus 0 gets forwarded to bus 2 and vice versa
static bool alloutput_passthrough = false;

static safety_config alloutput_init(uint16_t param) {
  // Enables passthrough mode where relay is open and bus 0 gets forwarded to bus 2 and vice versa
  const uint16_t ALLOUTPUT_PARAM_PASSTHROUGH = 1;
  controls_allowed = true;
  alloutput_passthrough = GET_FLAG(param, ALLOUTPUT_PARAM_PASSTHROUGH);
  return (safety_config){NULL, 0, NULL, 0};
}

static bool alloutput_tx_hook(const CANPacket_t *to_send) {
  UNUSED(to_send);
  return true;
}

static int alloutput_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;
  UNUSED(addr);

  if (alloutput_passthrough) {
    if (bus_num == 0) {
      bus_fwd = 2;
    }
    if (bus_num == 2) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

const safety_hooks alloutput_hooks = {
  .init = alloutput_init,
  .rx = default_rx_hook,
  .tx = alloutput_tx_hook,
  .fwd = alloutput_fwd_hook,
};
