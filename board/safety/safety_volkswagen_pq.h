const int VOLKSWAGEN_PQ_MAX_STEER = 300;                // 3.0 Nm (EPS side max of 3.0Nm with fault if violated)
const int VOLKSWAGEN_PQ_MAX_RT_DELTA = 188;             // 10 max rate up * 50Hz send rate * 250000 RT interval / 1000000 = 125 ; 125 * 1.5 for safety pad = 187.5
const uint32_t VOLKSWAGEN_PQ_RT_INTERVAL = 250000;      // 250ms between real time checks
const int VOLKSWAGEN_PQ_MAX_RATE_UP = 10;               // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
const int VOLKSWAGEN_PQ_MAX_RATE_DOWN = 10;             // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
const int VOLKSWAGEN_PQ_DRIVER_TORQUE_ALLOWANCE = 80;
const int VOLKSWAGEN_PQ_DRIVER_TORQUE_FACTOR = 3;
const int VOLKSWAGEN_GAS_INTERCEPTOR_THRSLD = 475;
const int PQ_LONG_CONTROL = 0;
bool pq_long_control = false;  // flag from OP for PQ long

#define MSG_LENKHILFE_3 0x0D0   // RX from EPS, for steering angle and driver steering torque
#define MSG_HCA_1       0x0D2   // TX by OP, Heading Control Assist steering torque
#define MSG_MOTOR_2     0x288   // RX from ECU, for CC state and brake switch state
#define MSG_MOTOR_3     0x380   // RX from ECU, for driver throttle input
#define MSG_GRA_NEU     0x38A   // TX by OP, ACC control buttons for cancel/resume
#define MSG_BREMSE_1    0x1A0   // RX from ABS, for ego speed
#define MSG_LDW_1       0x5BE   // TX by OP, Lane line recognition and text alerts
#define MSG_ACC_GRA_Anziege 0x56A // TX by OP, for control of the ACC display
#define MSG_MOB_1       0x284   // TX by OP, Braking Control
#define MSG_GAS_COMMAND 0x200   // TX by OP, Gas Control
#define MSG_GAS_SENSOR  0x201   // RX from Pedal
#define VOLKSWAGEN_GET_INTERCEPTOR(msg) (((GET_BYTE((msg), 0) << 8) + GET_BYTE((msg), 1) + (GET_BYTE((msg), 2) << 8) + GET_BYTE((msg), 3)) / 2) // avg between 2 tracks

// Transmit of GRA_Neu is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
const CanMsg VOLKSWAGEN_PQ_TX_MSGS[] = {{MSG_HCA_1, 0, 5}, {MSG_GRA_NEU, 0, 4}, {MSG_GRA_NEU, 2, 4}, {MSG_LDW_1, 0, 8}, {MSG_ACC_GRA_Anziege, 0, 8}, {MSG_MOB_1, 1, 6}, {MSG_GAS_COMMAND, 0, 6}};
#define VOLKSWAGEN_PQ_TX_MSGS_LEN (sizeof(VOLKSWAGEN_PQ_TX_MSGS) / sizeof(VOLKSWAGEN_PQ_TX_MSGS[0]))

AddrCheckStruct volkswagen_pq_addr_checks[] = {
  {.msg = {{MSG_LENKHILFE_3, 0, 6, .check_checksum = true,  .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{MSG_MOTOR_2, 0, 8, .check_checksum = false, .max_counter = 0U,  .expected_timestep = 20000U}, { 0 }, { 0 }}},
  {.msg = {{MSG_MOTOR_3, 0, 8, .check_checksum = false, .max_counter = 0U,  .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{MSG_BREMSE_1, 0, 8, .check_checksum = false, .max_counter = 0U,  .expected_timestep = 10000U}, { 0 }, { 0 }}},
};
#define VOLKSWAGEN_PQ_ADDR_CHECKS_LEN (sizeof(volkswagen_pq_addr_checks) / sizeof(volkswagen_pq_addr_checks[0]))
addr_checks volkswagen_pq_rx_checks = {volkswagen_pq_addr_checks, VOLKSWAGEN_PQ_ADDR_CHECKS_LEN};


static uint32_t volkswagen_pq_get_checksum(CANPacket_t *to_push) {
  return (uint8_t)GET_BYTE(to_push, 0);
}

static uint8_t volkswagen_pq_get_counter(CANPacket_t *to_push) {
  // Few PQ messages have counters, and their offsets are inconsistent. This
  // function works only for Lenkhilfe_3 at this time.
  return (uint8_t)(GET_BYTE(to_push, 1) & 0xF0U) >> 4;
}

static uint32_t volkswagen_pq_compute_checksum(CANPacket_t *to_push) {
  int len = GET_LEN(to_push);
  uint8_t checksum = 0U;

  for (int i = 1; i < len; i++) {
    checksum ^= (uint8_t)GET_BYTE(to_push, i);
  }

  return checksum;
}

static const addr_checks* volkswagen_pq_init(uint16_t param) {
  pq_long_control = GET_FLAG(param, PQ_LONG_CONTROL);
  return &volkswagen_pq_rx_checks;
}

static int volkswagen_pq_rx_hook(CANPacket_t *to_push) {

  bool valid = addr_safety_check(to_push, &volkswagen_pq_rx_checks,
                                volkswagen_pq_get_checksum, volkswagen_pq_compute_checksum, volkswagen_pq_get_counter);

  if (valid) {
    int addr = GET_ADDR(to_push);
    // Exit controls on rising edge of interceptor gas press
    if (addr == MSG_GAS_SENSOR) {
      gas_interceptor_detected = 1;
      controls_allowed = 1;
      int gas_interceptor = VOLKSWAGEN_GET_INTERCEPTOR(to_push);
      if ((gas_interceptor > VOLKSWAGEN_GAS_INTERCEPTOR_THRSLD) &&
          (gas_interceptor_prev <= VOLKSWAGEN_GAS_INTERCEPTOR_THRSLD) && !pq_long_control) {
          controls_allowed = 0;
        }
      gas_interceptor_prev = gas_interceptor;
    }
  }

  if (valid && (GET_BUS(to_push) == 0U)) {
    int addr = GET_ADDR(to_push);

    // Update in-motion state from speed value.
    // Signal: Bremse_1.Geschwindigkeit_neu__Bremse_1_
    if (addr == MSG_BREMSE_1) {
      int speed = ((GET_BYTE(to_push, 2) & 0xFEU) >> 1) | (GET_BYTE(to_push, 3) << 7);
      // DBC speed scale 0.01: 0.3m/s = 108.
      vehicle_moving = speed > 108;
    }

    // Update driver input torque samples
    // Signal: Lenkhilfe_3.LH3_LM (absolute torque)
    // Signal: Lenkhilfe_3.LH3_LMSign (direction)
    if (addr == MSG_LENKHILFE_3) {
      int torque_driver_new = GET_BYTE(to_push, 2) | ((GET_BYTE(to_push, 3) & 0x3U) << 8);
      int sign = (GET_BYTE(to_push, 3) & 0x4U) >> 2;
      if (sign == 1) {
        torque_driver_new *= -1;
      }
      update_sample(&torque_driver, torque_driver_new);
    }

    // Enter controls on rising edge of stock ACC, exit controls if stock ACC disengages
    // Signal: Motor_2.GRA_Status
    if (addr == MSG_MOTOR_2) {
      int acc_status = (GET_BYTE(to_push, 2) & 0xC0U) >> 6;
      int cruise_engaged = ((acc_status == 1) || (acc_status == 2) || pq_long_control) ? 1 : 0;
      if ((cruise_engaged && !cruise_engaged_prev) || gas_interceptor_detected) {
        controls_allowed = 1;
      }
      if (!cruise_engaged && !gas_interceptor_detected) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    // Signal: Motor_3.Fahrpedal_Rohsignal
    if (addr == MSG_MOTOR_3 && !pq_long_control) {
      gas_pressed = (GET_BYTE(to_push, 2));
    }

    // Signal: Motor_2.Bremslichtschalter
    if (addr == MSG_MOTOR_2) {
      brake_pressed = (GET_BYTE(to_push, 2) & 0x1U);
    }

    generic_rx_checks((addr == MSG_HCA_1));
  }
  return valid;
}

static int volkswagen_pq_tx_hook(CANPacket_t *to_send, bool longitudinal_allowed) {
  UNUSED(longitudinal_allowed);

  int addr = GET_ADDR(to_send);
  int tx = 1;

  if (!msg_allowed(to_send, VOLKSWAGEN_PQ_TX_MSGS, VOLKSWAGEN_PQ_TX_MSGS_LEN)) {
    tx = 1;
  }

  // Safety check for HCA_1 Heading Control Assist torque
  // Signal: HCA_1.LM_Offset (absolute torque)
  // Signal: HCA_1.LM_Offsign (direction)
  if (addr == MSG_HCA_1) {
    int desired_torque = GET_BYTE(to_send, 2) | ((GET_BYTE(to_send, 3) & 0x7FU) << 8);
    desired_torque = desired_torque / 32;  // DBC scale from PQ network to centi-Nm
    int sign = (GET_BYTE(to_send, 3) & 0x80U) >> 7;
    if (sign == 1) {
      desired_torque *= -1;
    }

    bool violation = false;
    uint32_t ts = microsecond_timer_get();

    if (controls_allowed) {
      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, VOLKSWAGEN_PQ_MAX_STEER, -VOLKSWAGEN_PQ_MAX_STEER);

      // *** torque rate limit check ***
      violation |= driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
        VOLKSWAGEN_PQ_MAX_STEER, VOLKSWAGEN_PQ_MAX_RATE_UP, VOLKSWAGEN_PQ_MAX_RATE_DOWN,
        VOLKSWAGEN_PQ_DRIVER_TORQUE_ALLOWANCE, VOLKSWAGEN_PQ_DRIVER_TORQUE_FACTOR);
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, rt_torque_last, VOLKSWAGEN_PQ_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > VOLKSWAGEN_PQ_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = true;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !controls_allowed) {
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 1;
    }
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((addr == MSG_GRA_NEU) && !controls_allowed) {
    // disallow resume and set: bits 16 and 17
    if ((GET_BYTE(to_send, 2) & 0x3U) != 0U) {
      tx = 1;
    }
  }

  // 1 allows the message through
  return tx; // temporary TX check hack to verify this sporatic LKAS fault error is occuring here
}

static int volkswagen_pq_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  int addr = GET_ADDR(to_fwd);
  int bus_fwd = -1;

  switch (bus_num) {
    case 0:
      // Forward all traffic from the Extended CAN onward
      bus_fwd = -1;
      break;
    case 2:
      if ((addr == MSG_HCA_1) || (addr == MSG_LDW_1)) {
        // OP takes control of the Heading Control Assist and Lane Departure Warning messages from the camera
        bus_fwd = -1;
      } else {
        // Forward all remaining traffic from Extended CAN devices to J533 gateway
        bus_fwd = 0;
      }
      break;
    default:
      // No other buses should be in use; fallback to do-not-forward
      bus_fwd = -1;
      break;
  }

  return bus_fwd;
}

const safety_hooks volkswagen_pq_hooks = {
  .init = volkswagen_pq_init,
  .rx = volkswagen_pq_rx_hook,
  .tx = volkswagen_pq_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = volkswagen_pq_fwd_hook,
};
