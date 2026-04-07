# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# Generate a baremetal application

PROGRAM = mmu_sv32_test
PROGRAM_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

EXTRA_SRCS := mmu_sv32_asm.S

include ${PROGRAM_DIR}/../common/common.mk
