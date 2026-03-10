.option norvc

    /* -----------------------------------------------------------------------
     * Common suite support (trap handlers, timer helpers, shared state)
     *
     * This file also contains the CSR-alias tests group.
     *
     * Control-flow design:
     * - All tests ultimately resume in M-mode at suite_resume_mepc.
     * - S-mode traps are validated in S-mode, then "escaped" to M-mode via ECALL.
     * - The M-mode trap handler forces MPP=M before mret so we always resume in M-mode.
     * - Trap handlers never call C. They only update suite_* globals.
     * ----------------------------------------------------------------------- */

    .section .text

    /* Memory map used by the provided tests */
    .equ MTIME_LO,       0x30000
    .equ MTIME_HI,       0x30004
    .equ MTIMECMP_LO,    0x30008
    .equ MTIMECMP_HI,    0x3000C

    .equ STIP_BIT,       0x20      /* bit 5 */
    .equ MTIP_BIT,       0x80      /* bit 7 */

    /* suite_expected_mode values */
    .equ SUITE_EXPECT_NONE, 0
    .equ SUITE_EXPECT_M,    1
    .equ SUITE_EXPECT_S,    2

    /* suite_result values */
    .equ SUITE_RESULT_PENDING, 0
    .equ SUITE_RESULT_PASS,    1
    .equ SUITE_RESULT_FAIL,    2

    /* suite_fail_reason values */
    .equ SUITE_FAIL_NONE,              0
    .equ SUITE_FAIL_UNEXPECTED_TRAP,   1
    .equ SUITE_FAIL_WRONG_TRAP_MODE,   2
    .equ SUITE_FAIL_CAUSE_MISMATCH,    3
    .equ SUITE_FAIL_EPC_MISMATCH,      4
    .equ SUITE_FAIL_TVAL_MISMATCH,     5
    .equ SUITE_FAIL_NO_TRAP_TIMEOUT,   6

    /* suite_last_trap_mode values */
    .equ SUITE_TRAPMODE_NONE, 0
    .equ SUITE_TRAPMODE_M,    1
    .equ SUITE_TRAPMODE_S,    2

    /* -----------------------------------------------------------------------
     * Shared suite state (read/written by tests, trap handlers, and C runner)
     * ----------------------------------------------------------------------- */
    .section .bss
    .align 4

    .global suite_expected_mode
suite_expected_mode:        .word 0
    .global suite_expected_cause
suite_expected_cause:       .word 0
    .global suite_expected_cause_valid
suite_expected_cause_valid: .word 0
    .global suite_expected_epc
suite_expected_epc:         .word 0
    .global suite_expected_epc_valid
suite_expected_epc_valid:   .word 0
    .global suite_expected_tval
suite_expected_tval:        .word 0
    .global suite_expected_tval_valid
suite_expected_tval_valid:  .word 0

    .global suite_resume_mepc
suite_resume_mepc:          .word 0

    .global suite_result
suite_result:               .word 0
    .global suite_fail_reason
suite_fail_reason:          .word 0

    .global suite_saved_ra
suite_saved_ra:             .word 0
    .global suite_saved_s0
suite_saved_s0:             .word 0
    .global suite_saved_s1
suite_saved_s1:             .word 0
    .global suite_saved_s2
suite_saved_s2:             .word 0
    .global suite_saved_sp
suite_saved_sp:             .word 0

    .global suite_last_trap_mode
suite_last_trap_mode:       .word 0
    .global suite_last_cause
suite_last_cause:           .word 0
    .global suite_last_epc
suite_last_epc:             .word 0
    .global suite_last_tval
suite_last_tval:            .word 0

    /* Internal escape flag:
     * Set by S-mode trap handler before executing ECALL to reach M-mode.
     * Consumed by M-mode trap handler. */
    .global suite_m_internal_escape
suite_m_internal_escape:    .word 0

    /* -----------------------------------------------------------------------
     * Trap stacks (used via mscratch/sscratch swap in trap entry)
     * ----------------------------------------------------------------------- */
    .p2align 4
    .global suite_m_trap_stack
suite_m_trap_stack:
    .space 512
    .global suite_m_trap_stack_top
suite_m_trap_stack_top:

    .p2align 4
    .global suite_s_trap_stack
suite_s_trap_stack:
    .space 512
    .global suite_s_trap_stack_top
suite_s_trap_stack_top:

    .section .text
    .align 4

    /* -----------------------------------------------------------------------
     * Helper: set mstatus.MPP = M and clear mstatus.MPIE so mret resumes in
     * M-mode with interrupts disabled.
     * ----------------------------------------------------------------------- */
    .macro SUITE_FORCE_RETURN_TO_M
        li t6, 0x1800          /* MPP bits */
        csrs mstatus, t6       /* MPP = 3 (M) */
        li t6, 0x80            /* MPIE */
        csrc mstatus, t6       /* MPIE = 0 -> MIE=0 after mret */
    .endm

    /* -----------------------------------------------------------------------
     * Helper: disable timer by moving mtimecmp far away.
     * Safe in any privilege level in this simple memory-mapped environment.
     * ----------------------------------------------------------------------- */
    .macro SUITE_DISABLE_TIMER
        li t6, MTIMECMP_HI
        li t5, -1
        sw t5, 0(t6)
        li t6, MTIMECMP_LO
        sw t5, 0(t6)
    .endm

    /* -----------------------------------------------------------------------
     * suite_setup_env()
     * Sets a known-safe baseline: interrupts off, delegation cleared, timer
     * disabled, suite trap handler vectors installed, trap stacks set up,
     * suite state cleared.
     * ----------------------------------------------------------------------- */
    .global suite_setup_env
suite_setup_env:
    addi sp, sp, -16
    sw ra, 12(sp)

    /* Disable all interrupt enables (machine and supervisor), and clear
       global enable bits. */
    li t0, -1
    csrc mie, t0
    csrc sie, t0
    csrc mstatus, t0

    /* Clear delegation registers */
    csrw mideleg, zero
    csrw medeleg, zero

    /* Disable timer */
    SUITE_DISABLE_TIMER

    /* Install direct-mode trap vectors */
    la t0, suite_m_trap_handler
    csrw mtvec, t0
    la t0, suite_s_trap_handler
    csrw stvec, t0

    /* Install trap stacks */
    la t0, suite_m_trap_stack_top
    csrw mscratch, t0
    la t0, suite_s_trap_stack_top
    csrw sscratch, t0

    /* Clear suite state */
    la t0, suite_expected_mode
    sw zero, 0(t0)
    la t0, suite_expected_cause
    sw zero, 0(t0)
    la t0, suite_expected_cause_valid
    sw zero, 0(t0)
    la t0, suite_expected_epc
    sw zero, 0(t0)
    la t0, suite_expected_epc_valid
    sw zero, 0(t0)
    la t0, suite_expected_tval
    sw zero, 0(t0)
    la t0, suite_expected_tval_valid
    sw zero, 0(t0)
    la t0, suite_resume_mepc
    sw zero, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)

    la t0, suite_last_trap_mode
    sw zero, 0(t0)
    la t0, suite_last_cause
    sw zero, 0(t0)
    la t0, suite_last_epc
    sw zero, 0(t0)
    la t0, suite_last_tval
    sw zero, 0(t0)

    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    lw ra, 12(sp)
    addi sp, sp, 16
    ret

    /* -----------------------------------------------------------------------
     * suite_cleanup_env()
     * Leaves suite_result and last-trap info intact, but disables interrupts,
     * clears delegation, and disables the timer so subsequent tests are not
     * affected.
     * ----------------------------------------------------------------------- */
    .global suite_cleanup_env
suite_cleanup_env:
    addi sp, sp, -16
    sw ra, 12(sp)

    li t0, -1
    csrc mie, t0
    csrc sie, t0
    csrw mideleg, zero
    csrw medeleg, zero

    /* Clear global interrupt enables for safety (MIE and SIE bits). */
    li t0, 0x0A             /* bits 3 (MIE) and 1 (SIE): 0b1010 */
    csrc mstatus, t0

    SUITE_DISABLE_TIMER

    /* Restore vectors to suite defaults (direct mode). */
    la t0, suite_m_trap_handler
    csrw mtvec, t0
    la t0, suite_s_trap_handler
    csrw stvec, t0

    lw ra, 12(sp)
    addi sp, sp, 16
    ret

    /* -----------------------------------------------------------------------
     * suite_program_timer_delta(delta)
     * Programs mtimecmp = mtime + delta, using the safe high/low update order.
     * a0 = delta (ticks)
     * ----------------------------------------------------------------------- */
    .global suite_program_timer_delta
suite_program_timer_delta:
    addi sp, sp, -16
    sw ra, 12(sp)

    li t0, MTIME_LO
    lw t1, 0(t0)
    li t0, MTIME_HI
    lw t2, 0(t0)

    add t1, t1, a0
    sltu t3, t1, a0
    add t2, t2, t3

    li t0, MTIMECMP_HI
    li t4, -1
    sw t4, 0(t0)
    li t0, MTIMECMP_LO
    sw t1, 0(t0)
    li t0, MTIMECMP_HI
    sw t2, 0(t0)

    lw ra, 12(sp)
    addi sp, sp, 16
    ret

    /* -----------------------------------------------------------------------
     * suite_timer_disable()
     * ----------------------------------------------------------------------- */
    .global suite_timer_disable
suite_timer_disable:
    SUITE_DISABLE_TIMER
    ret

    /* -----------------------------------------------------------------------
     * Save/restore the normal function caller context explicitly so tests do
     * not depend on trap-returned SP/RA state.
     * ----------------------------------------------------------------------- */
    .global suite_save_caller_context
suite_save_caller_context:
    la t0, suite_saved_ra
    lw t1, 12(sp)
    sw t1, 0(t0)
    la t0, suite_saved_sp
    sw sp, 0(t0)
    ret

    .global suite_restore_caller_context_and_return
suite_restore_caller_context_and_return:
    la t0, suite_saved_ra
    lw t1, 0(t0)
    la t0, suite_saved_sp
    lw sp, 0(t0)
    addi sp, sp, 16
    jr t1

    /* -----------------------------------------------------------------------
     * Trap handler: M-mode
     * ----------------------------------------------------------------------- */
    /* Work around the current core's trap-base truncation by placing the
       installed handlers on 256-byte boundaries. */
    .p2align 8
    .global suite_m_trap_handler
suite_m_trap_handler:
    .option push
    .option rvc
    c.j suite_m_trap_handler_body
    c.j suite_m_trap_handler_body
    .option pop
suite_m_trap_handler_body:
    /* Swap onto the dedicated M-mode trap stack. */
    csrrw sp, mscratch, sp
    addi sp, sp, -64

    sw ra, 0(sp)
    sw t0, 4(sp)
    sw t1, 8(sp)
    sw t2, 12(sp)
    sw t3, 16(sp)
    sw t4, 20(sp)
    sw t5, 24(sp)
    sw t6, 28(sp)
    sw a0, 32(sp)
    sw a1, 36(sp)
    sw a2, 40(sp)
    sw a3, 44(sp)
    sw a4, 48(sp)
    sw a5, 52(sp)
    sw a6, 56(sp)
    sw a7, 60(sp)

    /* Read trap CSRs early */
    csrr t0, mcause
    csrr t1, mepc
    csrr t2, mtval

    /* Controlled S->M return path: only consume the escape flag for an
       ECALL-from-S trap. Do not overwrite the recorded S-mode trap info. */
    la t3, suite_m_internal_escape
    lw t4, 0(t3)
    beqz t4, 1f
    li t5, 9                  /* ECALL from S-mode */
    bne t0, t5, 1f
    sw zero, 0(t3)

    /* Always resume in M-mode */
    SUITE_FORCE_RETURN_TO_M

    la t5, suite_resume_mepc
    lw t6, 0(t5)
    csrw mepc, t6

    j 9f

1:
    /* Record last-trap info (M-mode) */
    la t3, suite_last_trap_mode
    li t4, SUITE_TRAPMODE_M
    sw t4, 0(t3)
    la t3, suite_last_cause
    sw t0, 0(t3)
    la t3, suite_last_epc
    sw t1, 0(t3)
    la t3, suite_last_tval
    sw t2, 0(t3)

    /* Only treat real interrupt causes as timer traps. */
    li t3, 0x80000000
    and t4, t0, t3
    beqz t4, m_after_timer_check
    li t3, 0x7fffffff
    and t4, t0, t3
    li t5, 5
    beq t4, t5, m_timer_disable
    li t5, 7
    beq t4, t5, m_timer_disable
    j m_after_timer_check

m_timer_disable:
    SUITE_DISABLE_TIMER
    li t3, -1
    csrc mie, t3
    csrc sie, t3
    li t3, 0x0A
    csrc mstatus, t3

m_after_timer_check:
    /* Validate expectation */
    la t3, suite_expected_mode
    lw t4, 0(t3)
    beqz t4, unexpected_m

    li t5, SUITE_EXPECT_M
    beq t4, t5, expected_m

    /* Expected S-mode trap, but got M-mode */
    la t6, suite_result
    li t5, SUITE_RESULT_FAIL
    sw t5, 0(t6)
    la t6, suite_fail_reason
    li t5, SUITE_FAIL_WRONG_TRAP_MODE
    sw t5, 0(t6)
    j m_finish

expected_m:
    /* Check cause if requested */
    la t6, suite_expected_cause_valid
    lw t5, 0(t6)
    beqz t5, m_skip_cause
    la t6, suite_expected_cause
    lw t5, 0(t6)
    bne t0, t5, m_cause_bad
m_skip_cause:

    /* Check epc if requested */
    la t6, suite_expected_epc_valid
    lw t5, 0(t6)
    beqz t5, m_skip_epc
    la t6, suite_expected_epc
    lw t5, 0(t6)
    bne t1, t5, m_epc_bad
m_skip_epc:

    /* Check tval if requested */
    la t6, suite_expected_tval_valid
    lw t5, 0(t6)
    beqz t5, m_skip_tval
    la t6, suite_expected_tval
    lw t5, 0(t6)
    bne t2, t5, m_tval_bad
m_skip_tval:

    /* PASS */
    la t6, suite_result
    li t5, SUITE_RESULT_PASS
    sw t5, 0(t6)
    la t6, suite_fail_reason
    li t5, SUITE_FAIL_NONE
    sw t5, 0(t6)
    j m_finish

m_cause_bad:
    la t6, suite_result
    li t5, SUITE_RESULT_FAIL
    sw t5, 0(t6)
    la t6, suite_fail_reason
    li t5, SUITE_FAIL_CAUSE_MISMATCH
    sw t5, 0(t6)
    j m_finish

m_epc_bad:
    la t6, suite_result
    li t5, SUITE_RESULT_FAIL
    sw t5, 0(t6)
    la t6, suite_fail_reason
    li t5, SUITE_FAIL_EPC_MISMATCH
    sw t5, 0(t6)
    j m_finish

m_tval_bad:
    la t6, suite_result
    li t5, SUITE_RESULT_FAIL
    sw t5, 0(t6)
    la t6, suite_fail_reason
    li t5, SUITE_FAIL_TVAL_MISMATCH
    sw t5, 0(t6)
    j m_finish

unexpected_m:
    la t6, suite_result
    li t5, SUITE_RESULT_FAIL
    sw t5, 0(t6)
    la t6, suite_fail_reason
    li t5, SUITE_FAIL_UNEXPECTED_TRAP
    sw t5, 0(t6)
    j m_finish

m_finish:
    /* Always resume in M-mode at suite_resume_mepc */
    SUITE_FORCE_RETURN_TO_M

    la t5, suite_resume_mepc
    lw t6, 0(t5)
    csrw mepc, t6

9:
    /* Restore registers and return from trap */
    lw ra, 0(sp)
    lw t0, 4(sp)
    lw t1, 8(sp)
    lw t2, 12(sp)
    lw t3, 16(sp)
    lw t4, 20(sp)
    lw t5, 24(sp)
    lw t6, 28(sp)
    lw a0, 32(sp)
    lw a1, 36(sp)
    lw a2, 40(sp)
    lw a3, 44(sp)
    lw a4, 48(sp)
    lw a5, 52(sp)
    lw a6, 56(sp)
    lw a7, 60(sp)

    addi sp, sp, 64
    csrrw sp, mscratch, sp
    mret

    /* -----------------------------------------------------------------------
     * Trap handler: S-mode
     * ----------------------------------------------------------------------- */
    .p2align 8
    .global suite_s_trap_handler
suite_s_trap_handler:
    .option push
    .option rvc
    c.j suite_s_trap_handler_body
    c.j suite_s_trap_handler_body
    .option pop
suite_s_trap_handler_body:
    /* Swap onto the dedicated S-mode trap stack. */
    csrrw sp, sscratch, sp
    addi sp, sp, -64

    sw ra, 0(sp)
    sw t0, 4(sp)
    sw t1, 8(sp)
    sw t2, 12(sp)
    sw t3, 16(sp)
    sw t4, 20(sp)
    sw t5, 24(sp)
    sw t6, 28(sp)
    sw a0, 32(sp)
    sw a1, 36(sp)
    sw a2, 40(sp)
    sw a3, 44(sp)
    sw a4, 48(sp)
    sw a5, 52(sp)
    sw a6, 56(sp)
    sw a7, 60(sp)

    csrr t0, scause
    csrr t1, sepc
    csrr t2, stval

    /* Record last-trap info (S-mode) */
    la t3, suite_last_trap_mode
    li t4, SUITE_TRAPMODE_S
    sw t4, 0(t3)
    la t3, suite_last_cause
    sw t0, 0(t3)
    la t3, suite_last_epc
    sw t1, 0(t3)
    la t3, suite_last_tval
    sw t2, 0(t3)

    /* Only treat real interrupt causes as timer traps. */
    li t3, 0x80000000
    and t4, t0, t3
    beqz t4, s_after_timer_check
    li t3, 0x7fffffff
    and t4, t0, t3
    li t5, 5
    bne t4, t5, s_after_timer_check
    SUITE_DISABLE_TIMER
    li t3, -1
    csrc sie, t3
    li t3, 0x2
    csrc sstatus, t3

s_after_timer_check:
    /* Validate expectation */
    la t3, suite_expected_mode
    lw t4, 0(t3)
    beqz t4, unexpected_s

    li t5, SUITE_EXPECT_S
    beq t4, t5, expected_s

    /* Expected M-mode trap, but got S-mode */
    la t6, suite_result
    li t5, SUITE_RESULT_FAIL
    sw t5, 0(t6)
    la t6, suite_fail_reason
    li t5, SUITE_FAIL_WRONG_TRAP_MODE
    sw t5, 0(t6)
    j s_finish

expected_s:
    /* Check cause if requested */
    la t6, suite_expected_cause_valid
    lw t5, 0(t6)
    beqz t5, s_skip_cause
    la t6, suite_expected_cause
    lw t5, 0(t6)
    bne t0, t5, s_cause_bad
s_skip_cause:

    /* Check epc if requested */
    la t6, suite_expected_epc_valid
    lw t5, 0(t6)
    beqz t5, s_skip_epc
    la t6, suite_expected_epc
    lw t5, 0(t6)
    bne t1, t5, s_epc_bad
s_skip_epc:

    /* Check tval if requested */
    la t6, suite_expected_tval_valid
    lw t5, 0(t6)
    beqz t5, s_skip_tval
    la t6, suite_expected_tval
    lw t5, 0(t6)
    bne t2, t5, s_tval_bad
s_skip_tval:

    /* PASS */
    la t6, suite_result
    li t5, SUITE_RESULT_PASS
    sw t5, 0(t6)
    la t6, suite_fail_reason
    li t5, SUITE_FAIL_NONE
    sw t5, 0(t6)
    j s_finish

s_cause_bad:
    la t6, suite_result
    li t5, SUITE_RESULT_FAIL
    sw t5, 0(t6)
    la t6, suite_fail_reason
    li t5, SUITE_FAIL_CAUSE_MISMATCH
    sw t5, 0(t6)
    j s_finish

s_epc_bad:
    la t6, suite_result
    li t5, SUITE_RESULT_FAIL
    sw t5, 0(t6)
    la t6, suite_fail_reason
    li t5, SUITE_FAIL_EPC_MISMATCH
    sw t5, 0(t6)
    j s_finish

s_tval_bad:
    la t6, suite_result
    li t5, SUITE_RESULT_FAIL
    sw t5, 0(t6)
    la t6, suite_fail_reason
    li t5, SUITE_FAIL_TVAL_MISMATCH
    sw t5, 0(t6)
    j s_finish

unexpected_s:
    la t6, suite_result
    li t5, SUITE_RESULT_FAIL
    sw t5, 0(t6)
    la t6, suite_fail_reason
    li t5, SUITE_FAIL_UNEXPECTED_TRAP
    sw t5, 0(t6)

s_finish:
    /* Mark that the next trap to M-mode is an internal escape back to the suite. */
    la t5, suite_m_internal_escape
    li t6, 1
    sw t6, 0(t5)

    /* Restore registers and swap back to thread stack before escaping */
    lw ra, 0(sp)
    lw t0, 4(sp)
    lw t1, 8(sp)
    lw t2, 12(sp)
    lw t3, 16(sp)
    lw t4, 20(sp)
    lw t5, 24(sp)
    lw t6, 28(sp)
    lw a0, 32(sp)
    lw a1, 36(sp)
    lw a2, 40(sp)
    lw a3, 44(sp)
    lw a4, 48(sp)
    lw a5, 52(sp)
    lw a6, 56(sp)
    lw a7, 60(sp)

    addi sp, sp, 64
    csrrw sp, sscratch, sp

    /* Escape to M-mode. M handler will jump to suite_resume_mepc in M-mode. */
    ecall

    /* Should never reach here */
    j .

s_finish_resume_only:
    lw ra, 0(sp)
    lw t0, 4(sp)
    lw t1, 8(sp)
    lw t2, 12(sp)
    lw t3, 16(sp)
    lw t4, 20(sp)
    lw t5, 24(sp)
    lw t6, 28(sp)
    lw a0, 32(sp)
    lw a1, 36(sp)
    lw a2, 40(sp)
    lw a3, 44(sp)
    lw a4, 48(sp)
    lw a5, 52(sp)
    lw a6, 56(sp)
    lw a7, 60(sp)
    addi sp, sp, 64
    csrrw sp, sscratch, sp
    sret

    /* -----------------------------------------------------------------------
     * CSR alias tests group
     * ----------------------------------------------------------------------- */
    .section .rodata
    .align 4
csr_alias_group_name:
    .asciz "CSR alias tests"

test_name_mie_sie_alias:
    .asciz "mie/sie alias for delegated STIE"
test_desc_mie_sie_alias:
    .asciz "After delegating S-timer interrupt, mie.STIE and sie.STIE should read back identically."

test_name_mip_sip_alias:
    .asciz "mip/sip alias for pending STIP"
test_desc_mip_sip_alias:
    .asciz "After scheduling the timer, mip.STIP and sip.STIP should report the same pending bit."

    .section .text
    .align 4
    .extern suite_run_test

    .global smode_csr_alias_tests_run_all
smode_csr_alias_tests_run_all:
    addi sp, sp, -16
    sw ra, 12(sp)

    /* Group banner is printed by C main */
    la a0, test_name_mie_sie_alias
    la a1, test_desc_mie_sie_alias
    la a2, smode_test_mie_sie_alias
    call suite_run_test

    la a0, test_name_mip_sip_alias
    la a1, test_desc_mip_sip_alias
    la a2, smode_test_mip_sip_alias
    call suite_run_test

    lw ra, 12(sp)
    addi sp, sp, 16
    ret

    /* -----------------------------------------------------------------------
     * Test: mie/sie alias correctness (from tests.txt: 'mie/sie alias correctness')
     * ----------------------------------------------------------------------- */
    .global smode_test_mie_sie_alias
smode_test_mie_sie_alias:
    addi sp, sp, -16
    sw ra, 12(sp)

    call suite_setup_env

    /* We expect no traps in this test */
    la t0, suite_expected_mode
    sw zero, 0(t0)
    la t0, suite_expected_cause_valid
    sw zero, 0(t0)
    la t0, suite_expected_epc_valid
    sw zero, 0(t0)
    la t0, suite_expected_tval_valid
    sw zero, 0(t0)

    /* Ensure we can return if a trap happens unexpectedly */
    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    /* Delegate supervisor timer interrupt (STIP/STIE bit position used in provided tests) */
    li t0, STIP_BIT
    csrs mideleg, t0

    /* Write mie.STIE */
    li t0, STIP_BIT
    csrs mie, t0

    /* Read mie and sie and compare masked bits */
    csrr t1, mie
    csrr t2, sie
    li t3, STIP_BIT
    and t1, t1, t3
    and t2, t2, t3
    bne t1, t2, 2f

    /* PASS */
    la t0, suite_result
    li t1, SUITE_RESULT_PASS
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NONE
    sw t1, 0(t0)
    j 1f

2:
    /* FAIL */
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_UNEXPECTED_TRAP
    sw t1, 0(t0)

1:
    call suite_cleanup_env

    /* Return 0 on pass, 1 on fail */
    la t0, suite_result
    lw t1, 0(t0)
    li t2, SUITE_RESULT_PASS
    beq t1, t2, 3f
    li a0, 1
    j 4f
3:
    li a0, 0
4:
    lw ra, 12(sp)
    addi sp, sp, 16
    ret

    /* -----------------------------------------------------------------------
     * Test: mip/sip alias correctness (from tests.txt: 'mip/sip alias correctness')
     * Includes a timeout to avoid hanging if the timer never becomes pending.
     * ----------------------------------------------------------------------- */
    .global smode_test_mip_sip_alias
smode_test_mip_sip_alias:
    addi sp, sp, -16
    sw ra, 12(sp)

    call suite_setup_env

    /* We expect no traps in this test */
    la t0, suite_expected_mode
    sw zero, 0(t0)
    la t0, suite_expected_cause_valid
    sw zero, 0(t0)
    la t0, suite_expected_epc_valid
    sw zero, 0(t0)
    la t0, suite_expected_tval_valid
    sw zero, 0(t0)

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    /* Delegate STIP bit position used by the provided tests */
    li t0, STIP_BIT
    csrs mideleg, t0

    /* Program timer compare = now + 10000 */
    li a0, 10000
    call suite_program_timer_delta

    /* Wait (bounded) for pending bit to assert */
    li t4, 50000
wait_pending:
    csrr t0, mip
    li t1, STIP_BIT
    and t0, t0, t1
    bnez t0, pending_set
    addi t4, t4, -1
    bnez t4, wait_pending

    /* Timeout -> FAIL */
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NO_TRAP_TIMEOUT
    sw t1, 0(t0)
    j 1f

pending_set:
    /* Read mip and sip and compare masked pending bit */
    csrr t0, mip
    csrr t1, sip
    li t2, STIP_BIT
    and t0, t0, t2
    and t1, t1, t2
    bne t0, t1, alias_fail

    /* PASS */
    la t0, suite_result
    li t1, SUITE_RESULT_PASS
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NONE
    sw t1, 0(t0)
    j 1f

alias_fail:
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_UNEXPECTED_TRAP
    sw t1, 0(t0)

1:
    /* Disable timer so it doesn't affect later tests */
    call suite_timer_disable
    call suite_cleanup_env

    /* Return 0 on pass, 1 on fail */
    la t0, suite_result
    lw t1, 0(t0)
    li t2, SUITE_RESULT_PASS
    beq t1, t2, 2f
    li a0, 1
    j 3f
2:
    li a0, 0
3:
    lw ra, 12(sp)
    addi sp, sp, 16
    ret
