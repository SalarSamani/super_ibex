#include <stdint.h>

/*
 * Supervisor-mode test suite runner for Ibex.
 *
 * Key design points:
 * - The C runner prints all human-readable output.
 * - Assembly trap handlers never call C; they only update suite_* globals.
 * - Each test returns to C (0 = pass, nonzero = fail). Failures never stop the suite.
 *
 * The test behaviors are derived strictly from tests.txt.
 */

/* MMIO addresses used by the provided tests */
#define UART_ADDR     ((volatile uint8_t *)0x20000)
#define SIM_CTRL_ADDR ((volatile uint32_t *)0x20008)

/* -------------------------------------------------------------------------- */
/* Minimal UART printing (do not rely on libc)                                 */
/* -------------------------------------------------------------------------- */

static void suite_uart_putc(char c) {
  *UART_ADDR = (uint8_t)c;
}

static void suite_uart_puts(const char *s) {
  if (!s) return;
  while (*s) suite_uart_putc(*s++);
}

static void suite_uart_puts_ln(const char *s) {
  suite_uart_puts(s);
  suite_uart_putc('\n');
}

static void suite_uart_puthex32(uint32_t v) {
  static const char hex[] = "0123456789abcdef";
  suite_uart_puts("0x");
  for (int i = 7; i >= 0; --i) {
    uint8_t nib = (v >> (i * 4)) & 0xF;
    suite_uart_putc(hex[nib]);
  }
}

static void suite_uart_putdec_u32(uint32_t v) {
  /* Simple decimal printer for small counts (tests, pass/fail totals) */
  char buf[11];
  int idx = 0;
  if (v == 0) {
    suite_uart_putc('0');
    return;
  }
  while (v && idx < (int)sizeof(buf)) {
    buf[idx++] = (char)('0' + (v % 10));
    v /= 10;
  }
  for (int i = idx - 1; i >= 0; --i) suite_uart_putc(buf[i]);
}

/* -------------------------------------------------------------------------- */
/* Exports from assembly support                                               */
/* -------------------------------------------------------------------------- */

extern volatile uint32_t suite_expected_mode;
extern volatile uint32_t suite_expected_cause;
extern volatile uint32_t suite_expected_cause_valid;
extern volatile uint32_t suite_expected_epc;
extern volatile uint32_t suite_expected_epc_valid;
extern volatile uint32_t suite_expected_tval;
extern volatile uint32_t suite_expected_tval_valid;

extern volatile uint32_t suite_result;
extern volatile uint32_t suite_fail_reason;

extern volatile uint32_t suite_last_trap_mode;
extern volatile uint32_t suite_last_cause;
extern volatile uint32_t suite_last_epc;
extern volatile uint32_t suite_last_tval;

/* -------------------------------------------------------------------------- */
/* Group entry points (defined in the 4 assembly files)                        */
/* -------------------------------------------------------------------------- */

extern void smode_privilege_tests_run_all(void);
extern void smode_medeleg_tests_run_all(void);
extern void smode_interrupt_tests_run_all(void);
extern void smode_csr_alias_tests_run_all(void);
extern int smode_test_timer_not_delegated_goes_to_m(void);
extern int smode_test_timer_delegated_goes_to_s(void);
extern int smode_test_timer_mideleg_routing_pair(void);

/* -------------------------------------------------------------------------- */
/* Suite runner API called from assembly group files                           */
/* -------------------------------------------------------------------------- */

typedef int (*suite_test_fn_t)(void);

static uint32_t g_total = 0;
static uint32_t g_pass = 0;
static uint32_t g_fail = 0;
static volatile const char *g_current_test_name = 0;
static volatile const char *g_current_test_desc = 0;
static uint32_t g_finish_after_current_test = 0;

static void suite_finish_simulation(void);

/* Values must match the assembly .equ definitions */
enum {
  SUITE_EXPECT_NONE = 0,
  SUITE_EXPECT_M    = 1,
  SUITE_EXPECT_S    = 2,
};

enum {
  SUITE_RESULT_PENDING = 0,
  SUITE_RESULT_PASS    = 1,
  SUITE_RESULT_FAIL    = 2,
};

enum {
  SUITE_FAIL_NONE            = 0,
  SUITE_FAIL_UNEXPECTED_TRAP = 1,
  SUITE_FAIL_WRONG_TRAP_MODE = 2,
  SUITE_FAIL_CAUSE_MISMATCH  = 3,
  SUITE_FAIL_EPC_MISMATCH    = 4,
  SUITE_FAIL_TVAL_MISMATCH   = 5,
  SUITE_FAIL_NO_TRAP_TIMEOUT = 6,
};

enum {
  SUITE_TRAPMODE_NONE = 0,
  SUITE_TRAPMODE_M    = 1,
  SUITE_TRAPMODE_S    = 2,
};

static void suite_print_expectation(void) {
  suite_uart_puts("  Expectation: ");
  if (suite_expected_mode == SUITE_EXPECT_NONE) {
    suite_uart_puts("no trap expected\n");
    return;
  }
  if (suite_expected_mode == SUITE_EXPECT_M) suite_uart_puts("trap in M-mode");
  else if (suite_expected_mode == SUITE_EXPECT_S) suite_uart_puts("trap in S-mode");
  else suite_uart_puts("trap in <unknown mode>");

  suite_uart_putc('\n');

  if (suite_expected_cause_valid) {
    suite_uart_puts("    expected cause: ");
    suite_uart_puthex32(suite_expected_cause);
    suite_uart_putc('\n');
  }
  if (suite_expected_epc_valid) {
    suite_uart_puts("    expected epc  : ");
    suite_uart_puthex32(suite_expected_epc);
    suite_uart_putc('\n');
  }
  if (suite_expected_tval_valid) {
    suite_uart_puts("    expected tval : ");
    suite_uart_puthex32(suite_expected_tval);
    suite_uart_putc('\n');
  }
}

static void suite_print_last_trap(void) {
  suite_uart_puts("  Last trap observed:\n");

  suite_uart_puts("    mode : ");
  if (suite_last_trap_mode == SUITE_TRAPMODE_NONE) suite_uart_puts("none\n");
  else if (suite_last_trap_mode == SUITE_TRAPMODE_M) suite_uart_puts("M\n");
  else if (suite_last_trap_mode == SUITE_TRAPMODE_S) suite_uart_puts("S\n");
  else suite_uart_puts("unknown\n");

  suite_uart_puts("    cause: ");
  suite_uart_puthex32(suite_last_cause);
  suite_uart_putc('\n');

  suite_uart_puts("    epc  : ");
  suite_uart_puthex32(suite_last_epc);
  suite_uart_putc('\n');

  suite_uart_puts("    tval : ");
  suite_uart_puthex32(suite_last_tval);
  suite_uart_putc('\n');
}

static void suite_print_fail_reason(void) {
  suite_uart_puts("  Failure reason: ");
  switch (suite_fail_reason) {
    case SUITE_FAIL_UNEXPECTED_TRAP:
      suite_uart_puts_ln("unexpected trap occurred (no trap expected).");
      break;
    case SUITE_FAIL_WRONG_TRAP_MODE:
      suite_uart_puts_ln("trap taken in the wrong privilege mode.");
      break;
    case SUITE_FAIL_CAUSE_MISMATCH:
      suite_uart_puts_ln("trap cause mismatch.");
      break;
    case SUITE_FAIL_EPC_MISMATCH:
      suite_uart_puts_ln("trap EPC mismatch.");
      break;
    case SUITE_FAIL_TVAL_MISMATCH:
      suite_uart_puts_ln("trap TVAL mismatch.");
      break;
    case SUITE_FAIL_NO_TRAP_TIMEOUT:
      suite_uart_puts_ln("expected trap did not occur before the test timed out / continued past the faulting step.");
      break;
    default:
      if (suite_fail_reason == SUITE_FAIL_NONE) suite_uart_puts_ln("none (unexpected: test failed but fail_reason=0).");
      else {
        suite_uart_puts("unknown code ");
        suite_uart_putdec_u32(suite_fail_reason);
        suite_uart_putc('\n');
      }
      break;
  }
}

/*
 * Called by assembly "group runners" for each test case.
 * DO NOT call this from trap context.
 */
void suite_run_test(const char *name, const char *desc, suite_test_fn_t fn) {
  g_total++;
  g_current_test_name = name ? name : "<unnamed>";
  g_current_test_desc = desc ? desc : "";

  suite_uart_puts("\n[TEST START] ");
  suite_uart_puts((const char *)g_current_test_name);
  suite_uart_puts("\n  ");
  suite_uart_puts((const char *)g_current_test_desc);
  suite_uart_putc('\n');

  /* Run the test function. Pass/fail is owned by suite_result in memory. */
  int rc = fn ? fn() : 1;
  (void)rc;

  if (suite_result == SUITE_RESULT_PASS) {
    g_pass++;
    suite_uart_puts("[PASS] ");
    suite_uart_puts((const char *)g_current_test_name);
    suite_uart_putc('\n');
    if (g_finish_after_current_test) {
      suite_finish_simulation();
    }
    return;
  }

  g_fail++;
  suite_uart_puts("[FAIL] ");
  suite_uart_puts((const char *)g_current_test_name);
  suite_uart_putc('\n');

  if (suite_result == SUITE_RESULT_PENDING) {
    suite_uart_puts_ln("  Failure reason: test returned without committing suite_result.");
  }

  /* Print detailed diagnostics */
  suite_print_fail_reason();
  suite_print_expectation();
  suite_print_last_trap();
}

/* -------------------------------------------------------------------------- */
/* Main                                                                        */
/* -------------------------------------------------------------------------- */

static void suite_group_banner(const char *title) {
  suite_uart_puts("\n============================================================\n");
  suite_uart_puts("Group: ");
  suite_uart_puts(title);
  suite_uart_puts("\n============================================================\n");
}

static void suite_finish_simulation(void) {
  suite_uart_puts("\n============================================================\n");
  suite_uart_puts("Suite complete.\n");
  suite_uart_puts("  Total: ");
  suite_uart_putdec_u32(g_total);
  suite_uart_puts("\n  Pass : ");
  suite_uart_putdec_u32(g_pass);
  suite_uart_puts("\n  Fail : ");
  suite_uart_putdec_u32(g_fail);
  suite_uart_puts("\n============================================================\n");

  /* Stop the simulation using the same mechanism the original tests used. */
  *SIM_CTRL_ADDR = 1;

  while (1) {
    __asm__ volatile("wfi");
  }
}

int main(void) {
  suite_uart_puts("\n============================================================\n");
  suite_uart_puts("Ibex supervisor-mode test suite (multi-test framework)\n");
  suite_uart_puts("============================================================\n");
  suite_uart_puts("This run will execute all tests, even if some fail.\n");

  suite_group_banner("Privilege restriction (TSR/TVM)");
  smode_privilege_tests_run_all();

  suite_group_banner("Exception delegation (medeleg)");
  smode_medeleg_tests_run_all();

  suite_group_banner("Interrupt/timer behavior (mideleg)");
  g_finish_after_current_test = 1;
  suite_run_test("Timer interrupt routing with and without mideleg",
                 "Checks both cases: mideleg clear routes timer to M-mode, and mideleg[STIP]=1 routes timer to S-mode.",
                 smode_test_timer_mideleg_routing_pair);
  g_finish_after_current_test = 0;

  suite_finish_simulation();
  return 0;
}
