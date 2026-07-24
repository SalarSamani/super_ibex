// Copyright 2026.
// SPDX-License-Identifier: Apache-2.0

#include "disk.h"
#include "simple_system_common.h"   // DEV_READ, DEV_WRITE

// Internal helper: program the address registers.
static void disk_setup(uint32_t sector, uint32_t count, const void *buf) {
  DEV_WRITE(DISK_SECTOR_LO, sector);
  DEV_WRITE(DISK_SECTOR_HI, 0u);          // 32-bit LBA, hi always 0
  DEV_WRITE(DISK_COUNT,    count);
  DEV_WRITE(DISK_BUF_ADDR, (uint32_t)(uintptr_t)buf);
}

// Internal helper: poll STATUS until it is no longer BUSY.
static uint32_t disk_wait(void) {
  uint32_t s;
  do {
    s = DEV_READ(DISK_STATUS, 0);
  } while (s == DISK_STATUS_BUSY);
  return s;
}

int disk_read(uint32_t dev, uint32_t sector, uint32_t count, uint8_t *buf) {
  (void)dev;
  if (count == 0u || buf == 0) return -1;
  disk_setup(sector, count, buf);
  DEV_WRITE(DISK_CMD, DISK_CMD_READ);     // kick the peripheral
  return (disk_wait() == DISK_STATUS_OK) ? 0 : -1;
}

int disk_write(uint32_t dev, uint32_t sector, uint32_t count, uint8_t *buf) {
  (void)dev;
  if (count == 0u || buf == 0) return -1;
  disk_setup(sector, count, buf);
  DEV_WRITE(DISK_CMD, DISK_CMD_WRITE);
  return (disk_wait() == DISK_STATUS_OK) ? 0 : -1;
}