#ifndef HALMET_SERIAL_H_
#define HALMET_SERIAL_H_

#include <esp_mac.h>

#include <cstdint>

// The base MAC as a 48-bit number, most significant byte first, so the
// device-specific (NIC) bytes land in the low bits.
//
// tNMEA2000::SetDeviceInformation keeps only the low 21 bits of the unique
// number. Packing the MAC least-significant-byte-first put MAC bytes 0..2 --
// the manufacturer OUI, identical across a production batch -- in those bits,
// so every HALMET with the same OUI claimed the same NMEA 2000 unique number
// (and, with the same function/instance, the same NAME), leading to address
// claim conflicts with two units on one bus.
inline uint64_t GetBoardSerialNumber() {
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  uint64_t serial = 0;
  for (int i = 0; i < 6; i++) {
    serial = (serial << 8) | mac[i];
  }
  return serial;
}

#endif  // HALMET_SERIAL_H_
