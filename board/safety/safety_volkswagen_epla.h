#pragma once

#define BUS_0 0  // car CAN
#define BUS_1 1  // unused for now
#define BUS_2 2  // EPS CAN

#define PLA_1           0x3D4
#define BREMSE_1        0x1A0
#define BREMSE_3        0x4A0
#define KOMBI_1         0x320

bool filter;
int counter;
int pla_stat;

/*
msg name   signal name          B0-7  sb0-7  len1-X
mBremse_1: BR1_Rad_kmh          2     1      15
mBremse_3: BR3_Fahrtr_VL        0     0      1
           BR3_Rad_kmh_VL       0     1      15
           BR3_Fahrtr_VR        2     0      1
           BR3_Rad_kmh_VR       2     1      15
           BR3_Fahrtr_HL        4     0      1
           BR3_Rad_kmh_HL       4     1      15
           BR3_Fahrtr_HR        6     0      1
           BR3_Rad_kmh_HR       6     1      15
mKombi_1:  KO1_kmh              3     1      15
*/

static void epla_rx_hook(const CANPacket_t* to_push) {
  const int bus_num = GET_BUS(to_push);
  const int addr = GET_ADDR(to_push);

    // if PLA isnt seen for 0.5s filter force cancels
  if (counter >= 20) {
    filter = 0;
    counter = 25;  // cap variable so we dont increment into infinity
  }

  switch (bus_num) {
    case BUS_0:
      if (addr == PLA_1) {
          // toggle filter on when PLA RX is status 4, or 6
        pla_stat = (GET_BYTE(to_push, 1) & 0b1111);
        filter = (pla_stat == 4U || pla_stat == 6U);
      }
      if (addr == KOMBI_1) {
        counter = filter ? 0 : +1;
      }
      break;
    case BUS_1:
      break;
    case BUS_2:
      break;
    default:
      break;
  }
}

static bool epla_tx_hook(const CANPacket_t* to_send) {
  UNUSED(to_send);
  return true;
}

static int epla_fwd_hook(int bus_num, int addr) {
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

const safety_hooks vw_pq_epla_hooks = {
  .init = alloutput_init,
  .rx = epla_rx_hook,
  .tx = epla_tx_hook,
  .fwd = epla_fwd_hook,
                  // bring these into file here
  .get_counter = volkswagen_pq_get_counter,
  .get_checksum = volkswagen_pq_get_checksum,
  .compute_checksum = volkswagen_pq_compute_checksum,
};