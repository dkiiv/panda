#pragma once

static void eepb_rx_hook(const CANPacket_t* to_push) {
  const int bus = GET_BUS(to_push);
  const int addr = GET_ADDR(to_push);
}

static bool eepb_tx_hook(const CANPacket_t* to_send) {
  UNUSED(to_send);
  return true;
}

static int eepb_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  switch (bus_num) {
    case 0:
      bus_fwd = 2;
      break;
    case 2:
      bus_fwd = 0;
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
