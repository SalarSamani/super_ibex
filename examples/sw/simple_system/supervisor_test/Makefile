PROGRAM = supervisor_test
PROGRAM_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

# Multi-file supervisor-mode test suite:
# - supervisor_test.c (C runner)
# - Four assembly files grouped by topic
EXTRA_SRCS := \
  smode_support_and_csr_alias_tests.S \
  smode_privilege_tests.S \
  smode_medeleg_tests.S \
  smode_interrupt_tests.S

include ${PROGRAM_DIR}/../common/common.mk