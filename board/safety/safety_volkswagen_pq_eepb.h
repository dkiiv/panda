#pragma once

#define BUS_0 0  // ext-CAN gateway, OEM EPB if present will be placed here
#define BUS_1 1  // pt-CAN
#define BUS_2 2  // ext-CAN radar

#define EPB_1           0x5C0
#define MOTOR_2         0x288
#define MOTOR_3         0x380
#define MOTOR_BREMSE    0x284
#define BREMSE_1        0x1A0
#define BREMSE_8        0x1AC
#define BREMSE_11       0x5B7
#define GRA_NEU         0x38A
#define ACC_SYSTEM      0x368
#define ACC_GRA_ANZEIGE 0x56A

// init all bytes
mEPB_1           EP1    = {0};
mMotor_2         MO2    = {0};
mBremse_8        B8     = {0};
mBremse_11       B11    = {0};
mGRA_Neu         GRA    = {0};
mACC_System      ACS    = {0};
mACC_GRA_Anzeige ACA    = {0};
CarState         CS     = {0};
ModuleState      self   = {0};

static void eepb_rx_hook(const CANPacket_t* to_push) {
  const int bus_num = GET_BUS(to_push);
  const int addr = GET_ADDR(to_push);

  switch (bus_num) {
    case BUS_0:
      if (addr == MOTOR_2) {
        CS.brakePressed = (GET_BYTE(to_push, 2) >> 7) & 0b1;
        for (int i = 0; i < GET_LEN(to_push); i++)
          MO2.msg[i] = GET_BYTE(to_push, i);
      }
      if (addr == MOTOR_3) {
        CS.gasPressed = GET_BYTE(to_push, 2) & 0xFF;
      }
      if (addr == GRA_NEU) {
        CS.cruiseCancel = (GET_BYTE(to_push, 1) >> 6) & 0b1;
        for (int i = 0; i < GET_LEN(to_push); i++)
          GRA.msg[i] = GET_BYTE(to_push, i);
      }
      if (addr == EPB_1) {              // OEM EPB module state
        CS.EP1_Freigabe_Ver = (GET_BYTE(to_push, 4) >> 6) & 0b1;
        CS.EP1_switchState  = (GET_BYTE(to_push, 1) >> 1) & 0b11;
        for (int i = 0; i < GET_LEN(to_push); i++)
          EP1.OEM[i] = GET_BYTE(to_push, i);
      }
      if (addr == BREMSE_1) {
        CS.vEgo = (((GET_BYTE(to_push, 2) & 0b1111111) << 8) | GET_BYTE(to_push, 3)) * 0.01;
      }
      if (addr == BREMSE_8) {
        for (int i = 0; i < GET_LEN(to_push); i++)
          B8.msg[i] = GET_BYTE(to_push, i);
      }
      if (addr == BREMSE_11) {
        for (int i = 0; i < GET_LEN(to_push); i++)
          B11.msg[i] = GET_BYTE(to_push, i);
      }
      break;
    case BUS_1:
      if (addr == MOTOR_BREMSE) {
        CS.MOB_Standby = (GET_BYTE(to_push, 1) >> 3) & 0b1;
      }
      if (addr == BREMSE_1) {
        self.frame = (self.frame + 1) % 1000;  // 100hz
      }
      break;
    case BUS_2:
      if (addr == ACC_SYSTEM) {
        ACS.Anhaltewunsch = (GET_BYTE(to_push, 4) >> 1) & 0b1;
        ACS.Sta_ADR = (GET_BYTE(to_push, 1) >> 2) & 0b11;
        ACS.StSt_Info = (GET_BYTE(to_push, 2) >> 6) & 0b11;
        ACS.FreigSollB = GET_BYTE(to_push, 2) & 0b1;
        ACS.Sollbeschl = ((GET_BYTE(to_push, 3) << 3) | ((GET_BYTE(to_push, 4) >> 5) & 0b111)) & 0b11111111111;
        self.stopping = ACS.Anhaltewunsch && (CS.vEgo <= 2 || self.stopping);
        self.stopped = self.EPB_enable && (CS.vEgo == 0 || (self.stopping && self.stopped));
        CS.aEgo = (ACS.Sollbeschl * 0.005) - 7.22;
        EPB_handler(&CS, &self);
        EP1.COUNTER = (EP1.COUNTER + 1) % 16;
        for (int i = 0; i < GET_LEN(to_push); i++)
          ACS.msg[i] = GET_BYTE(to_push, i);
      }
      if (addr == ACC_GRA_ANZEIGE) {
        for (int i = 0; i < GET_LEN(to_push); i++)
          ACA.msg[i] = GET_BYTE(to_push, i);
      }
      break;
    default:
      break;
  }
}

static bool eepb_tx_hook(const CANPacket_t* to_send) {
  UNUSED(to_send);
  return true;
}

static int eepb_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  switch (bus_num) {
    case BUS_0:
      bus_fwd = BUS_2;
      break;
    case BUS_2:
      bus_fwd = BUS_0;
      break;
    default:
      bus_fwd = -1;
      break;
  }

  return bus_fwd;
}

const safety_hooks vw_pq_eepb_hooks = {
  .init = alloutput_init,
  .rx = eepb_rx_hook,
  .tx = eepb_tx_hook,
  .fwd = eepb_fwd_hook,
                  // bring these into file here
  .get_counter = volkswagen_pq_get_counter,
  .get_checksum = volkswagen_pq_get_checksum,
  .compute_checksum = volkswagen_pq_compute_checksum,
};
