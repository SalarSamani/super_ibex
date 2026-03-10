.option norvc

    .section .text
    .align 4

    .extern suite_run_test
    .extern suite_setup_env
    .extern suite_cleanup_env

    .extern suite_expected_mode
    .extern suite_expected_cause
    .extern suite_expected_cause_valid
    .extern suite_expected_epc
    .extern suite_expected_epc_valid
    .extern suite_expected_tval
    .extern suite_expected_tval_valid
    .extern suite_resume_mepc
    .extern suite_result
    .extern suite_fail_reason
    .extern suite_m_internal_escape

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
test_name_medeleg_break_u2s:
    .asciz "medeleg[3] delegates U-mode EBREAK to S-mode"
test_desc_medeleg_break_u2s:
    .asciz "With medeleg[3]=1, an ebreak executed in U-mode must trap to S-mode (scause=3)."

test_name_medeleg_ecall_u2s:
    .asciz "medeleg[8] delegates U-mode ECALL to S-mode"
test_desc_medeleg_ecall_u2s:
    .asciz "With medeleg[8]=1, an ecall executed in U-mode must trap to S-mode (scause=8, stval=0)."

test_name_medeleg_ecall_u2m:
    .asciz "No medeleg: U-mode ECALL traps to M-mode"
test_desc_medeleg_ecall_u2m:
    .asciz "With medeleg[8]=0, an ecall executed in U-mode must trap to M-mode (mcause=8, mtval=0)."

test_name_medeleg_illegal_u2s:
    .asciz "medeleg[2] delegates U-mode illegal instruction to S-mode"
test_desc_medeleg_illegal_u2s:
    .asciz "With medeleg[2]=1, an illegal instruction in U-mode must trap to S-mode (scause=2, stval=0xffffffff)."

test_name_medeleg_illegal_u2m:
    .asciz "No medeleg: U-mode illegal instruction traps to M-mode"
test_desc_medeleg_illegal_u2m:
    .asciz "With medeleg[2]=0, an illegal instruction in U-mode must trap to M-mode (mcause=2, mtval=0xffffffff)."

test_name_break_in_s_stays_s:
    .asciz "EBREAK taken in S-mode stays in S-mode"
test_desc_break_in_s_stays_s:
    .asciz "Even if medeleg[3]=1, an ebreak executed in S-mode must be handled in S-mode (scause=3)."

test_name_nested_escalation:
    .asciz "Delegated ECALL enters S-mode, then illegal CSR read in S escalates to M"
test_desc_nested_escalation:
    .asciz "After a delegated U-mode ecall traps into S, an illegal 'csrr mcause' in S must trap to M (mcause=2)."

    .section .text
    .align 4

    .global smode_medeleg_tests_run_all
smode_medeleg_tests_run_all:
    addi sp, sp, -16
    sw ra, 12(sp)

    la a0, test_name_medeleg_break_u2s
    la a1, test_desc_medeleg_break_u2s
    la a2, smode_test_medeleg_ebreak_u_to_s
    call suite_run_test

    la a0, test_name_medeleg_ecall_u2s
    la a1, test_desc_medeleg_ecall_u2s
    la a2, smode_test_medeleg_ecall_u_to_s
    call suite_run_test

    la a0, test_name_medeleg_ecall_u2m
    la a1, test_desc_medeleg_ecall_u2m
    la a2, smode_test_ecall_u_to_m_no_medeleg
    call suite_run_test

    la a0, test_name_medeleg_illegal_u2s
    la a1, test_desc_medeleg_illegal_u2s
    la a2, smode_test_medeleg_illegal_u_to_s
    call suite_run_test

    la a0, test_name_medeleg_illegal_u2m
    la a1, test_desc_medeleg_illegal_u2m
    la a2, smode_test_illegal_u_to_m_no_medeleg
    call suite_run_test

    la a0, test_name_break_in_s_stays_s
    la a1, test_desc_break_in_s_stays_s
    la a2, smode_test_breakpoint_in_s_stays_in_s
    call suite_run_test

    la a0, test_name_nested_escalation
    la a1, test_desc_nested_escalation
    la a2, smode_test_delegated_ecall_then_s_illegal_escalates_to_m
    call suite_run_test

    lw ra, 12(sp)
    addi sp, sp, 16
    li a0, 0
    ret

    /* -----------------------------------------------------------------------
     * medeleg[3] delegates U-mode breakpoint (EBREAK) to S-mode (tests.txt: "medeleg")
     * ----------------------------------------------------------------------- */
    .global smode_test_medeleg_ebreak_u_to_s
smode_test_medeleg_ebreak_u_to_s:
    addi sp, sp, -16
    sw ra, 12(sp)

    call suite_setup_env

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    /* Expect trap in S-mode: scause=3, sepc = u_ebreak */
    la t0, suite_expected_mode
    li t1, SUITE_EXPECT_S
    sw t1, 0(t0)

    la t0, suite_expected_cause_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_cause
    li t1, 3
    sw t1, 0(t0)

    la t0, suite_expected_epc_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_epc
    la t1, u_ebreak
    sw t1, 0(t0)

    la t0, suite_expected_tval_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_tval
    sw zero, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    /* Delegate breakpoint exception (cause 3) to S-mode */
    li t0, (1 << 3)
    csrw medeleg, t0

    /* Enter U-mode */
    li t0, 0x1800
    csrc mstatus, t0
    la t0, u_entry_break
    csrw mepc, t0
    mret

u_entry_break:
u_ebreak:
    ebreak
    /* Should not continue */
    j u_fail_no_trap

u_fail_no_trap:
    /* If we reach here, no trap happened. Mark fail and try to escape via ecall. */
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NO_TRAP_TIMEOUT
    sw t1, 0(t0)
    la t0, suite_expected_mode
    sw zero, 0(t0)

    /* ECALL from U should always trap; even if it routes wrong, suite will collect */
    ecall
    j .

1:
    call suite_cleanup_env
    /* Return */
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

    /* -----------------------------------------------------------------------
     * medeleg[8] delegates U-mode ECALL to S-mode (tests.txt: multiple ECALL-to-S variants)
     * ----------------------------------------------------------------------- */
    .global smode_test_medeleg_ecall_u_to_s
smode_test_medeleg_ecall_u_to_s:
    addi sp, sp, -16
    sw ra, 12(sp)

    call suite_setup_env

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    la t0, suite_expected_mode
    li t1, SUITE_EXPECT_S
    sw t1, 0(t0)

    la t0, suite_expected_cause_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_cause
    li t1, 8                   /* ECALL from U */
    sw t1, 0(t0)

    la t0, suite_expected_epc_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_epc
    la t1, u_ecall
    sw t1, 0(t0)

    la t0, suite_expected_tval_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_tval
    sw zero, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    /* Delegate ECALL from U-mode */
    li t0, (1 << 8)
    csrw medeleg, t0

    /* Enter U-mode */
    li t0, 0x1800
    csrc mstatus, t0
    la t0, u_entry_ecall
    csrw mepc, t0
    mret

u_entry_ecall:
    nop
u_ecall:
    ecall
    j u_fail_no_trap_2

u_fail_no_trap_2:
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NO_TRAP_TIMEOUT
    sw t1, 0(t0)
    la t0, suite_expected_mode
    sw zero, 0(t0)
    ecall
    j .

1:
    call suite_cleanup_env
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

    /* -----------------------------------------------------------------------
     * No medeleg[8]: U-mode ECALL traps to M-mode (tests.txt: "ECALL ... when medeleg[8]=0")
     * ----------------------------------------------------------------------- */
    .global smode_test_ecall_u_to_m_no_medeleg
smode_test_ecall_u_to_m_no_medeleg:
    addi sp, sp, -16
    sw ra, 12(sp)

    call suite_setup_env

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    la t0, suite_expected_mode
    li t1, SUITE_EXPECT_M
    sw t1, 0(t0)

    la t0, suite_expected_cause_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_cause
    li t1, 8                   /* ECALL from U */
    sw t1, 0(t0)

    la t0, suite_expected_epc_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_epc
    la t1, u_ecall_3
    sw t1, 0(t0)

    la t0, suite_expected_tval_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_tval
    sw zero, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    /* Do NOT delegate ECALL */
    csrw medeleg, zero

    /* Enter U-mode */
    li t0, 0x1800
    csrc mstatus, t0
    la t0, u_entry_ecall_3
    csrw mepc, t0
    mret

u_entry_ecall_3:
u_ecall_3:
    ecall
    /* Must not continue */
    j u_fail_no_trap_3

u_fail_no_trap_3:
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NO_TRAP_TIMEOUT
    sw t1, 0(t0)
    la t0, suite_expected_mode
    sw zero, 0(t0)
    ecall
    j .

1:
    call suite_cleanup_env
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

    /* -----------------------------------------------------------------------
     * medeleg[2] delegates U-mode illegal instruction to S-mode (tests.txt: "illegal instruction ... medeleg[2]=1")
     * ----------------------------------------------------------------------- */
    .global smode_test_medeleg_illegal_u_to_s
smode_test_medeleg_illegal_u_to_s:
    addi sp, sp, -16
    sw ra, 12(sp)

    call suite_setup_env

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    la t0, suite_expected_mode
    li t1, SUITE_EXPECT_S
    sw t1, 0(t0)

    la t0, suite_expected_cause_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_cause
    li t1, 2                  /* illegal instruction */
    sw t1, 0(t0)

    la t0, suite_expected_epc_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_epc
    la t1, u_illegal_word
    sw t1, 0(t0)

    la t0, suite_expected_tval_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_tval
    li t1, -1                 /* 0xffffffff */
    sw t1, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    /* Delegate illegal instruction */
    li t0, (1 << 2)
    csrw medeleg, t0

    /* Enter U-mode */
    li t0, 0x1800
    csrc mstatus, t0
    la t0, u_entry_illegal
    csrw mepc, t0
    mret

u_entry_illegal:
u_illegal_word:
    .word 0xffffffff
    j u_fail_no_trap_4

u_fail_no_trap_4:
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NO_TRAP_TIMEOUT
    sw t1, 0(t0)
    la t0, suite_expected_mode
    sw zero, 0(t0)
    ecall
    j .

1:
    call suite_cleanup_env
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

    /* -----------------------------------------------------------------------
     * No medeleg: U-mode illegal instruction traps to M-mode (tests.txt: "illegal instruction ... medeleg[2]=0")
     * ----------------------------------------------------------------------- */
    .global smode_test_illegal_u_to_m_no_medeleg
smode_test_illegal_u_to_m_no_medeleg:
    addi sp, sp, -16
    sw ra, 12(sp)

    call suite_setup_env

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    la t0, suite_expected_mode
    li t1, SUITE_EXPECT_M
    sw t1, 0(t0)

    la t0, suite_expected_cause_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_cause
    li t1, 2
    sw t1, 0(t0)

    la t0, suite_expected_epc_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_epc
    la t1, u_illegal_word_2
    sw t1, 0(t0)

    la t0, suite_expected_tval_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_tval
    li t1, -1
    sw t1, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    csrw medeleg, zero

    /* Enter U-mode */
    li t0, 0x1800
    csrc mstatus, t0
    la t0, u_entry_illegal_2
    csrw mepc, t0
    mret

u_entry_illegal_2:
u_illegal_word_2:
    .word 0xffffffff
    j u_fail_no_trap_5

u_fail_no_trap_5:
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NO_TRAP_TIMEOUT
    sw t1, 0(t0)
    la t0, suite_expected_mode
    sw zero, 0(t0)
    ecall
    j .

1:
    call suite_cleanup_env
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

    /* -----------------------------------------------------------------------
     * Breakpoint within S-mode stays in S-mode even with medeleg[3]=1
     * (tests.txt: "even with medeleg[3]=1, a breakpoint taken in S-mode stays in S-mode")
     * ----------------------------------------------------------------------- */
    .global smode_test_breakpoint_in_s_stays_in_s
smode_test_breakpoint_in_s_stays_in_s:
    addi sp, sp, -16
    sw ra, 12(sp)

    call suite_setup_env

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    la t0, suite_expected_mode
    li t1, SUITE_EXPECT_S
    sw t1, 0(t0)

    la t0, suite_expected_cause_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_cause
    li t1, 3
    sw t1, 0(t0)

    la t0, suite_expected_epc_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_epc
    la t1, s_ebreak
    sw t1, 0(t0)

    la t0, suite_expected_tval_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_tval
    sw zero, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    li t0, (1 << 3)
    csrw medeleg, t0

    /* Enter S-mode */
    li t0, 0x1800
    csrc mstatus, t0
    li t0, 0x800
    csrs mstatus, t0
    la t0, s_entry_break
    csrw mepc, t0
    mret

s_entry_break:
s_ebreak:
    ebreak
    j s_fail_no_trap

s_fail_no_trap:
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
    call suite_cleanup_env
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

    /* -----------------------------------------------------------------------
     * Delegated ECALL enters S-mode; illegal CSR read inside the S trap handler escalates to M-mode
     * (tests.txt: "delegated ECALL from U enters S-mode, then a second trap ... escalates to M-mode")
     *
     * This uses a custom S trap handler for this test only.
     * ----------------------------------------------------------------------- */
    .global smode_test_delegated_ecall_then_s_illegal_escalates_to_m
smode_test_delegated_ecall_then_s_illegal_escalates_to_m:
    addi sp, sp, -16
    sw ra, 12(sp)

    call suite_setup_env

    /* Install custom S trap handler (direct mode) */
    la t0, s_trap_handler_nested
    csrw stvec, t0

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    /* First expected trap: S-mode ECALL from U */
    la t0, suite_expected_mode
    li t1, SUITE_EXPECT_S
    sw t1, 0(t0)
    la t0, suite_expected_cause_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_cause
    li t1, 8
    sw t1, 0(t0)
    la t0, suite_expected_epc_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_epc
    la t1, u_ecall_nested
    sw t1, 0(t0)
    la t0, suite_expected_tval_valid
    li t1, 1
    sw t1, 0(t0)
    la t0, suite_expected_tval
    sw zero, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    /* Delegate ECALL from U-mode only */
    li t0, (1 << 8)
    csrw medeleg, t0

    /* Enter U-mode */
    li t0, 0x1800
    csrc mstatus, t0
    la t0, u_entry_nested
    csrw mepc, t0
    mret

u_entry_nested:
u_ecall_nested:
    ecall
    j u_fail_no_trap_nested

u_fail_no_trap_nested:
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NO_TRAP_TIMEOUT
    sw t1, 0(t0)
    la t0, suite_expected_mode
    sw zero, 0(t0)
    ecall
    j .

    /* Custom S trap handler:
     * - validates first delegated ECALL using the normal suite expectation exercise
     * - sets up next expected trap: M-mode illegal instruction (cause=2) from 'csrr mcause' executed in S-mode
     * - triggers illegal CSR read to force escalation to M-mode
     */
    .p2align 8
s_trap_handler_nested:
    .option push
    .option rvc
    c.j s_trap_handler_nested_body
    c.j s_trap_handler_nested_body
    .option pop
s_trap_handler_nested_body:
    /* Minimal prologue: reuse the suite S handler's stack swap and save, but keep it local */
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

    /* Check the expected first trap directly */
    li t3, 8
    bne t0, t3, nested_fail
    la t3, u_ecall_nested
    bne t1, t3, nested_fail
    bnez t2, nested_fail

    /* Now set expectation for the second trap in M-mode: illegal instruction (cause=2).
       We don't overlay expected epc/tval for robustness, as original test only checked mcause. */
    la t3, suite_expected_mode
    li t4, SUITE_EXPECT_M
    sw t4, 0(t3)
    la t3, suite_expected_cause_valid
    li t4, 1
    sw t4, 0(t3)
    la t3, suite_expected_cause
    li t4, 2
    sw t4, 0(t3)
    la t3, suite_expected_epc_valid
    sw zero, 0(t3)
    la t3, suite_expected_tval_valid
    sw zero, 0(t3)

    /* Restore regs and return to thread stack before executing illegal CSR read,
       so the M-mode trap handler returns with a valid thread stack pointer. */
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

nested_illegal_csrr:
    /* Illegal in S-mode: should trap to M-mode (mcause=2) */
    csrr t0, mcause

    /* If we got here, escalation did not happen */
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

nested_fail:
    /* Mark failure and escape back to M-mode */
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_UNEXPECTED_TRAP
    sw t1, 0(t0)
    la t0, suite_expected_mode
    sw zero, 0(t0)

    /* Restore regs */
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

    la t0, suite_m_internal_escape
    li t1, 1
    sw t1, 0(t0)
    ecall
    j .

1:
    /* If the M trap from the illegal CSR read occurred and matched, suite_m_trap_handler marks PASS and resumes here. */
    call suite_cleanup_env
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
