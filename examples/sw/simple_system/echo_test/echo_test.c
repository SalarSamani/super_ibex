// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "simple_system_common.h"

int main(int argc, char **argv) {
  int c;
  while ((c = getchar()) != -1) {
    putchar(c);
  }
  sim_halt();
  return 0;
}
