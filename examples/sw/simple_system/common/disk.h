// Copyright 2026.
// SPDX-License-Identifier: Apache-2.0

#ifndef DISK_H_
#define DISK_H_

#include <stdint.h>

// Base of the disk MMIO window. Matches cfg_device_addr_base[Disk]
// in ibex_simple_system.sv.
#define DISK_BASE        0x00040000

// Register offsets inside the window.
#define DISK_CMD         (DISK_BASE + 0x00)
#define DISK_SECTOR_LO   (DISK_BASE + 0x04)
#define DISK_SECTOR_HI   (DISK_BASE + 0x08)
#define DISK_COUNT       (DISK_BASE + 0x0C)
#define DISK_BUF_ADDR    (DISK_BASE + 0x10)
#define DISK_STATUS      (DISK_BASE + 0x14)

// Command codes.
#define DISK_CMD_READ    1u
#define DISK_CMD_WRITE   2u

// Status codes.
#define DISK_STATUS_OK     0u
#define DISK_STATUS_ERROR  1u
#define DISK_STATUS_BUSY   2u

// Standard 512-byte sectors.
#define DISK_SECTOR_BYTES  512u

// Read `count` sectors starting at `sector` from device `dev` into `buf`.
// Returns 0 on success, non-zero on error. The buffer must be in
// simulated RAM (i.e. inside 0x00100000..0x001FFFFF) and aligned to 4.
int disk_read (uint32_t dev, uint32_t sector, uint32_t count, uint8_t *buf);

// Write `count` sectors of `buf` starting at `sector` of device `dev`.
// Same constraints as disk_read. Returns 0 on success.
int disk_write(uint32_t dev, uint32_t sector, uint32_t count, uint8_t *buf);

#endif // DISK_H_