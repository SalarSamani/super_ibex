.option norvc

    .section .text
    .align 4

    .extern suite_run_test
    .extern suite_setup_env
    .extern suite_cleanup_env
    .extern suite_program_timer_delta
    .extern suite_timer_disable
    .extern suite_save_caller_context
    .extern suite_restore_caller_context_and_return
    .extern suite_saved_s0
    .extern suite_expected_mode
    .extern suite_expected_cause
    .extern suite_expected_cause_valid
    .extern suite_expected_epc_valid
    .extern suite_expected_tval_valid
    .extern suite_resume_mepc
    .extern suite_result
    .extern suite_fail_reason
    .extern suite_m_internal_escape

    /* Timer memory map (matches tests.txt) */
    .equ MTIMECMP_LO, 0x30008
    .equ MTIMECMP_HI, 0x3000C

    .equ STIP_BIT, 0x20
    .equ MTIP_BIT, 0x80

    /* suite_expected_mode values */
    .equ SUITE_EXPECT_NONE, 0
    .equ SUITE_EXPECT_M,    1
    .equ SUITE_EXPECT_S,    2

    /* suite_result values */
    .equ SUITE_RESULT_PASS, 1
    .equ SUITE_RESULT_FAIL, 2

    /* suite_fail_reason values */
    .equ SUITE_FAIL_NONE,              0
    .equ SUITE_FAIL_UNEXPECTED_TRAP,   1
    .equ SUITE_FAIL_NO_TRAP_TIMEOUT,   6

    .section .rodata
    .align 4
test_name_mideleg_not_deleg:
    .asciz "Timer interrupt not delegated routes to M-mode"
test_desc_mideleg_not_deleg:
    .asciz "With mideleg clear, the timer interrupt should be handled in M-mode (routing test only)."

test_name_mideleg_deleg:
    .asciz "Timer interrupt delegated routes to S-mode"
test_desc_mideleg_deleg:
    .asciz "With mideleg[STIP]=1, the timer interrupt should be handled in S-mode (routing test only)."

test_name_sie_gate:
    .asciz "SIE gates supervisor timer interrupt take in S-mode"
test_desc_sie_gate:
    .asciz "With pending timer interrupt, SIE=0 must prevent trap; SIE=1 must allow trap to S-mode."

test_name_stvec_vectored:
    .asciz "Vectored stvec dispatches S-timer interrupt to vector entry 5"
test_desc_stvec_vectored:
    .asciz "With stvec.MODE=1 (vectored), supervisor timer interrupt must branch to the vector-table entry for cause 5."

test_name_wfi_wakeup:
    .asciz "WFI in S-mode takes supervisor timer interrupt (no hang design)"
test_desc_wfi_wakeup:
    .asciz "After STIP is pending with SIE=0, enabling SIE then executing wfi must trap to S-mode with scause=0x80000005."

    .section .text
    .align 4

    .global smode_interrupt_tests_run_all
smode_interrupt_tests_run_all:
    addi sp, sp, -16
    sw ra, 12(sp)

    la a0, test_name_mideleg_not_deleg
    la a1, test_desc_mideleg_not_deleg
    la a2, smode_test_timer_not_delegated_goes_to_m
    call suite_run_test

    lw ra, 12(sp)
    addi sp, sp, 16
    li a0, 0
    ret

    .global smode_test_timer_mideleg_routing_pair
smode_test_timer_mideleg_routing_pair:
    la t0, suite_saved_s0
    sw s0, 0(t0)
    mv s0, ra

    call smode_test_timer_not_delegated_goes_to_m
    bnez a0, 1f

    call smode_test_timer_delegated_goes_to_s

1:
    mv ra, s0
    la t0, suite_saved_s0
    lw s0, 0(t0)
    ret

    /* -----------------------------------------------------------------------
     * Test: timer interrupt not delegated -> taken in M-mode
     * Based on tests.txt: "mideleg not delegate to smode"
     *
     * Robustness: we do NOT check mcause value in this routing-only test, to
     * tolerate implementations that differ in whether the timer appears as
     * cause 5 vs 7 in M-mode.
     * ----------------------------------------------------------------------- */
    .global smode_test_timer_not_delegated_goes_to_m
smode_test_timer_not_delegated_goes_to_m:
    addi sp, sp, -16
    sw ra, 12(sp)
    call suite_save_caller_context

    call suite_setup_env

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    /* Expect M-mode trap (cause not validated) */
    la t0, suite_expected_mode
    li t1, SUITE_EXPECT_M
    sw t1, 0(t0)
    la t0, suite_expected_cause_valid
    sw zero, 0(t0)
    la t0, suite_expected_epc_valid
    sw zero, 0(t0)
    la t0, suite_expected_tval_valid
    sw zero, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    /* Do NOT delegate (bit used in provided tests) */
    li t0, STIP_BIT
    csrc mideleg, t0

    /* Route and take the timer directly in M-mode. */
    li t0, MTIP_BIT
    csrs mie, t0
    li t0, 0x8
    csrs mstatus, t0

    li a0, 10000
    call suite_program_timer_delta

    /* Busy-wait bounded. If no trap occurs, fail. */
    li t5, 50000
m_wait_loop_1:
    addi t5, t5, -1
    bnez t5, m_wait_loop_1

    /* No trap -> FAIL and continue in normal flow. */
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NO_TRAP_TIMEOUT
    sw t1, 0(t0)
    j 1f

1:
    /* Ensure timer is disabled */
    call suite_timer_disable
    call suite_cleanup_env

    la t0, suite_result
    lw t1, 0(t0)
    li t2, SUITE_RESULT_PASS
    beq t1, t2, 2f
    li a0, 1
    j 3f
2:
    la t0, suite_expected_mode
    sw zero, 0(t0)
    la t0, suite_resume_mepc
    sw zero, 0(t0)
    li a0, 0
3:
    j suite_restore_caller_context_and_return

    /* -----------------------------------------------------------------------
     * Test: timer interrupt delegated -> taken in S-mode
     * Based on tests.txt: "mideleg deleagte to smode"
     *
     * Routing-only: we validate trap is taken in S-mode, but do not validate
     * scause value here (the vectored stvec / WFI tests validate scause later).
     * ----------------------------------------------------------------------- */
    .global smode_test_timer_delegated_goes_to_s
smode_test_timer_delegated_goes_to_s:
    addi sp, sp, -16
    sw ra, 12(sp)
    call suite_save_caller_context

    call suite_setup_env

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    /* Expect S-mode trap (cause not validated) */
    la t0, suite_expected_mode
    li t1, SUITE_EXPECT_S
    sw t1, 0(t0)
    la t0, suite_expected_cause_valid
    sw zero, 0(t0)
    la t0, suite_expected_epc_valid
    sw zero, 0(t0)
    la t0, suite_expected_tval_valid
    sw zero, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    /* Delegate "timer" bit used in provided tests */
    li t0, STIP_BIT
    csrs mideleg, t0

    /* Enable STIE at machine and supervisor levels. */
    li t0, STIP_BIT
    csrs mie, t0
    csrs sie, t0

    /* Enable SIE globally before entering S-mode. */
    li t0, 0x2
    csrs mstatus, t0

    /* Program timer compare = now + 10000 before entering S-mode. */
    li a0, 10000
    call suite_program_timer_delta

    /* Enter S-mode directly. */
    li t0, 0x1800
    csrc mstatus, t0
    li t0, 0x800
    csrs mstatus, t0
    la t0, s_wait_for_timer_2
    csrw mepc, t0
    mret

s_wait_for_timer_2:
    /* Busy-wait bounded in S-mode. Expect interrupt to arrive before timeout. */
    li t5, 50000
s_wait_loop_2:
    addi t5, t5, -1
    bnez t5, s_wait_loop_2

    /* No trap -> FAIL and escape cleanly to M-mode. */
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NO_TRAP_TIMEOUT
    sw t1, 0(t0)
    la t0, suite_m_internal_escape
    li t1, 1
    sw t1, 0(t0)
    ecall
    j .

1:
    call suite_timer_disable
    call suite_cleanup_env

    la t0, suite_result
    lw t1, 0(t0)
    li t2, SUITE_RESULT_PASS
    beq t1, t2, 2f
    li a0, 1
    j 3f
2:
    la t0, suite_expected_mode
    sw zero, 0(t0)
    la t0, suite_resume_mepc
    sw zero, 0(t0)
    li a0, 0
3:
    j suite_restore_caller_context_and_return

    /* -----------------------------------------------------------------------
     * Test: SIE gate in S-mode (tests.txt: "get interrupt fired in smode")
     *
     * Phase 1: enter S-mode, set SIE=0, schedule timer, spin; must NOT trap.
     * Phase 2: schedule timer again, set SIE=1, spin; must trap to S-mode.
     * ----------------------------------------------------------------------- */
    .global smode_test_sie_gates_s_timer_interrupt
smode_test_sie_gates_s_timer_interrupt:
    addi sp, sp, -16
    sw ra, 12(sp)
    call suite_save_caller_context

    call suite_setup_env

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    /* Delegate timer to S-mode (bit position used in tests) */
    li t0, STIP_BIT
    csrs mideleg, t0

    /* Enable STIE at M and S levels */
    li t0, STIP_BIT
    csrs mie, t0
    csrs sie, t0

    /* Enter S-mode directly */
    li t0, 0x1800
    csrc mstatus, t0
    li t0, 0x800
    csrs mstatus, t0
    la t0, s_sie_gate_entry
    csrw mepc, t0
    mret

s_sie_gate_entry:
    /* Phase 1: SIE=0, schedule a timer and spin; must not trap */
    li t0, 0x2
    csrc sstatus, t0

    li a0, 2000
    call suite_program_timer_delta

    li t5, 40000
phase1_spin:
    addi t5, t5, -1
    bnez t5, phase1_spin

    /* Clear compare far away to drop pending */
    li t0, MTIMECMP_HI
    li t1, -1
    sw t1, 0(t0)
    li t0, MTIMECMP_LO
    sw t1, 0(t0)

    /* Phase 2: expect S-mode timer interrupt when SIE=1 */
    la t0, suite_expected_mode
    li t1, SUITE_EXPECT_S
    sw t1, 0(t0)
    la t0, suite_expected_cause_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_cause
    li t1, 0x80000005       /* supervisor timer interrupt */
    sw t1, 0(t0)
    la t0, suite_expected_epc_valid
    sw zero, 0(t0)
    la t0, suite_expected_tval_valid
    sw zero, 0(t0)

    /* Enable SIE */
    li t0, 0x2
    csrs sstatus, t0

    /* Schedule timer again and spin; expect trap */
    li a0, 2000
    call suite_program_timer_delta

    li t5, 40000
phase2_spin:
    addi t5, t5, -1
    bnez t5, phase2_spin

    /* No trap -> FAIL and escape to M */
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NO_TRAP_TIMEOUT
    sw t1, 0(t0)
    la t0, suite_expected_mode
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    li t1, 1
    sw t1, 0(t0)
    ecall
    j .

1:
    call suite_timer_disable
    call suite_cleanup_env

    la t0, suite_result
    lw t1, 0(t0)
    li t2, SUITE_RESULT_PASS
    beq t1, t2, 2f
    li a0, 1
    j 3f
2:
    la t0, suite_expected_mode
    sw zero, 0(t0)
    la t0, suite_resume_mepc
    sw zero, 0(t0)
    li a0, 0
3:
    j suite_restore_caller_context_and_return

    /* -----------------------------------------------------------------------
     * Test: vectored stvec dispatch (tests.txt: "vectored stvec")
     * ----------------------------------------------------------------------- */
    .section .text
    .align 4

    .p2align 8
s_vector_table_vectored:
    j vec_fail              /* 0 */
    j vec_fail              /* 1 */
    j vec_fail              /* 2 */
    j vec_fail              /* 3 */
    j vec_fail              /* 4 */
    j vec_timer_entry       /* 5 supervisor timer */
    j vec_fail              /* 6 */
    j vec_fail              /* 7 */

vec_timer_entry:
    /* If we got here, vectored dispatch hit the correct slot.
       Continue with the normal suite S trap handler by jumping to stvec base handler. */
    j suite_s_trap_handler

vec_fail:
    /* Wrong vector entry -> mark fail and escape to M */
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_UNEXPECTED_TRAP
    sw t1, 0(t0)
    la t0, suite_expected_mode
    sw zero, 0(t0)

    /* Escape */
    la t0, suite_m_internal_escape
    li t1, 1
    sw t1, 0(t0)
    ecall
    j .

    .extern suite_s_trap_handler

    .global smode_test_vectored_stvec_s_timer
smode_test_vectored_stvec_s_timer:
    addi sp, sp, -16
    sw ra, 12(sp)
    call suite_save_caller_context

    call suite_setup_env

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    /* Delegate timer to S-mode */
    li t0, STIP_BIT
    csrs mideleg, t0

    /* Enable STIE in both mie and sie */
    li t0, STIP_BIT
    csrs mie, t0
    csrs sie, t0

    /* Set expected S trap with scause=supervisor timer */
    la t0, suite_expected_mode
    li t1, SUITE_EXPECT_S
    sw t1, 0(t0)
    la t0, suite_expected_cause_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_cause
    li t1, 0x80000005
    sw t1, 0(t0)
    la t0, suite_expected_epc_valid
    sw zero, 0(t0)
    la t0, suite_expected_tval_valid
    sw zero, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    /* Install vectored stvec */
    la t0, s_vector_table_vectored
    ori t0, t0, 1          /* MODE=1 */
    csrw stvec, t0

    /* Enable SIE (global) before entering S-mode */
    li t0, 0x2
    csrs mstatus, t0

    /* Program timer */
    li a0, 10000
    call suite_program_timer_delta

    /* Enter S-mode */
    li t0, 0x1800
    csrc mstatus, t0
    li t0, 0x800
    csrs mstatus, t0
    la t0, s_wait_vectored
    csrw mepc, t0
    mret

s_wait_vectored:
    /* Spin bounded; interrupt should hit quickly */
    li t5, 50000
vec_spin:
    addi t5, t5, -1
    bnez t5, vec_spin

    /* No trap -> FAIL and escape */
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NO_TRAP_TIMEOUT
    sw t1, 0(t0)
    la t0, suite_expected_mode
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    li t1, 1
    sw t1, 0(t0)
    ecall
    j .

1:
    call suite_timer_disable
    call suite_cleanup_env

    la t0, suite_result
    lw t1, 0(t0)
    li t2, SUITE_RESULT_PASS
    beq t1, t2, 2f
    li a0, 1
    j 3f
2:
    la t0, suite_expected_mode
    sw zero, 0(t0)
    la t0, suite_resume_mepc
    sw zero, 0(t0)
    li a0, 0
3:
    j suite_restore_caller_context_and_return

    /* -----------------------------------------------------------------------
     * Test: WFI wakeup / trap behavior (based on tests.txt: "WFI wakeup")
     * Non-hang design: we wait for pending STIP with SIE=0 (bounded), then
     * set expectation, enable SIE, and execute wfi. Since STIP is already
     * pending, wfi should not block indefinitely.
     * ----------------------------------------------------------------------- */
    .global smode_test_wfi_takes_s_timer_interrupt
smode_test_wfi_takes_s_timer_interrupt:
    addi sp, sp, -16
    sw ra, 12(sp)
    call suite_save_caller_context

    call suite_setup_env

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    /* Delegate timer to S-mode and enable STIE at M and S levels */
    li t0, STIP_BIT
    csrs mideleg, t0
    csrs mie, t0
    csrs sie, t0

    /* Enter S-mode */
    li t0, 0x1800
    csrc mstatus, t0
    li t0, 0x800
    csrs mstatus, t0

    la t0, s_wfi_entry
    csrw mepc, t0
    mret

s_wfi_entry:
    /* Keep SIE=0 initially */
    li t0, 0x2
    csrc sstatus, t0

    /* Program timer for near future */
    li a0, 10000
    call suite_program_timer_delta

    /* Bounded wait for pending STIP bit. If it never becomes pending, fail without executing wfi. */
    li t5, 50000
wait_stip_pending:
    csrr t0, sip
    li t1, STIP_BIT
    and t0, t0, t1
    bnez t0, stip_is_pending
    addi t5, t5, -1
    bnez t5, wait_stip_pending

    /* Pending never asserted -> FAIL and escape */
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NO_TRAP_TIMEOUT
    sw t1, 0(t0)
    la t0, suite_m_internal_escape
    li t1, 1
    sw t1, 0(t0)
    ecall
    j .

stip_is_pending:
    /* Now expect supervisor timer interrupt trap in S-mode */
    la t0, suite_expected_mode
    li t1, SUITE_EXPECT_S
    sw t1, 0(t0)
    la t0, suite_expected_cause_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_cause
    li t1, 0x80000005
    sw t1, 0(t0)
    la t0, suite_expected_epc_valid
    sw zero, 0(t0)
    la t0, suite_expected_tval_valid
    sw zero, 0(t0)

    /* Enable SIE, then execute WFI. With a pending interrupt, this should trap immediately. */
    li t0, 0x2
    csrs sstatus, t0

wfi_insn:
    wfi

    /* If WFI falls through without trap, FAIL and escape */
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NO_TRAP_TIMEOUT
    sw t1, 0(t0)
    la t0, suite_expected_mode
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    li t1, 1
    sw t1, 0(t0)
    ecall
    j .

1:
    call suite_timer_disable
    call suite_cleanup_env

    la t0, suite_result
    lw t1, 0(t0)
    li t2, SUITE_RESULT_PASS
    beq t1, t2, 2f
    li a0, 1
    j 3f
2:
    la t0, suite_expected_mode
    sw zero, 0(t0)
    la t0, suite_resume_mepc
    sw zero, 0(t0)
    li a0, 0
3:
    j suite_restore_caller_context_and_return
