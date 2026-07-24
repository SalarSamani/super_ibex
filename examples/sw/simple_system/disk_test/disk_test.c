// Copyright 2026.
// SPDX-License-Identifier: Apache-2.0

#include <stdint.h>
#include <stddef.h>

#include "simple_system_common.h"
#include "disk.h"

#define NUM_SECTORS 3
#define BUF_SIZE    (NUM_SECTORS * DISK_SECTOR_BYTES)

// Big buffer in BSS — lives in RAM at a 4-byte-aligned address.
static uint8_t write_buf[BUF_SIZE] __attribute__((aligned(4)));
static uint8_t read_buf [BUF_SIZE] __attribute__((aligned(4)));

static void fill_pattern(uint8_t *b, uint32_t sector_base) {
  for (uint32_t s = 0; s < NUM_SECTORS; s++) {
    for (uint32_t i = 0; i < DISK_SECTOR_BYTES; i++) {
      b[s * DISK_SECTOR_BYTES + i] =
          (uint8_t)((sector_base + s) * 0x37u + i * 0x5Bu);
    }
  }
}

static int compare(const uint8_t *a, const uint8_t *b, uint32_t n) {
  for (uint32_t i = 0; i < n; i++) {
    if (a[i] != b[i]) return (int)i + 1;  // 1-based mismatch index
  }
  return 0;
}

int main(int argc, char **argv) {
  (void)argc; (void)argv;

  puts("disk_test: starting\n");

  // --- 1. Build a known-pattern buffer for sectors 0..2 -----------------
  fill_pattern(write_buf, 0);

  // --- 2. Write it to the disk ------------------------------------------
  puts("disk_test: writing sectors 0..2\n");
  if (disk_write(0, 0, NUM_SECTORS, write_buf) != 0) {
    puts("disk_test: WRITE FAILED\n");
    return 1;
  }

  // --- 3. Zero the read buffer to be sure ------------------------------
  for (uint32_t i = 0; i < BUF_SIZE; i++) read_buf[i] = 0xAAu;

  // --- 4. Read it back --------------------------------------------------
  puts("disk_test: reading sectors 0..2\n");
  if (disk_read(0, 0, NUM_SECTORS, read_buf) != 0) {
    puts("disk_test: READ FAILED\n");
    return 1;
  }

  // --- 5. Compare -------------------------------------------------------
  int mism = compare(write_buf, read_buf, BUF_SIZE);
  if (mism == 0) {
    puts("disk_test: PASS\n");
  } else {
    puts("disk_test: FAIL at byte 0x");
    puthex((uint32_t)(mism - 1));
    putchar('\n');
    return 2;
  }

  return 0;
}