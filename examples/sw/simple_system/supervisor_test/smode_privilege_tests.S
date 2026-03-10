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
    .equ SUITE_FAIL_NONE,            0
    .equ SUITE_FAIL_UNEXPECTED_TRAP, 1
    .equ SUITE_FAIL_NO_TRAP_TIMEOUT, 6

    .section .rodata
    .align 4
test_name_tsr_sret:
    .asciz "mstatus.TSR blocks SRET in S-mode"
test_desc_tsr_sret:
    .asciz "With TSR=1, executing sret in S-mode must trap (illegal instruction) to M-mode."

test_name_sret_mmode:
    .asciz "SRET executes in M-mode even with TSR=1"
test_desc_sret_mmode:
    .asciz "With TSR=1, executing sret in M-mode should not trap; it should deliver control to sepc."

test_name_tvm_sfence:
    .asciz "mstatus.TVM blocks SFENCE.VMA in S-mode"
test_desc_tvm_sfence:
    .asciz "With TVM=1, executing sfence.vma in S-mode must trap (illegal instruction) to M-mode."

test_name_tvm_satp:
    .asciz "mstatus.TVM blocks SATP access in S-mode"
test_desc_tvm_satp:
    .asciz "With TVM=1, accessing satp in S-mode must trap (illegal instruction) to M-mode."

    .section .text
    .align 4

    .global smode_privilege_tests_run_all
smode_privilege_tests_run_all:
    addi sp, sp, -16
    sw ra, 12(sp)

    la a0, test_name_tsr_sret
    la a1, test_desc_tsr_sret
    la a2, smode_test_tsr_blocks_sret
    call suite_run_test

    la a0, test_name_sret_mmode
    la a1, test_desc_sret_mmode
    la a2, smode_test_sret_in_mmode_allowed
    call suite_run_test

    la a0, test_name_tvm_sfence
    la a1, test_desc_tvm_sfence
    la a2, smode_test_tvm_blocks_sfence_vma
    call suite_run_test

    la a0, test_name_tvm_satp
    la a1, test_desc_tvm_satp
    la a2, smode_test_tvm_blocks_satp_access
    call suite_run_test

    lw ra, 12(sp)
    addi sp, sp, 16
    li a0, 0
    ret

    /* -----------------------------------------------------------------------
     * Test: TSR blocks SRET in S-mode (merged from tests.txt: "TSR test" and
     * the later unlabeled TSR variant).
     * ----------------------------------------------------------------------- */
    .global smode_test_tsr_blocks_sret
smode_test_tsr_blocks_sret:
    addi sp, sp, -16
    sw ra, 12(sp)

    call suite_setup_env

    /* resume point for any trap */
    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    /* Expect: M-mode illegal instruction at the SRET instruction address */
    la t0, suite_expected_mode
    li t1, SUITE_EXPECT_M
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
    la t1, tsr_sret_insn
    sw t1, 0(t0)

    la t0, suite_expected_tval_valid
    sw zero, 0(t0)

    /* Clear per-test status */
    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    /* Set TSR=1 */
    li t0, (1 << 22)
    csrs mstatus, t0

    /* Enter S-mode: MPP = 01 */
    li t0, 0x1800
    csrc mstatus, t0
    li t0, 0x800
    csrs mstatus, t0

    la t0, tsr_smode_entry
    csrw mepc, t0
    mret

tsr_smode_entry:
    /* If SRET does not trap (unexpected), keep execution in S-mode */
    la t0, tsr_after_sret
    csrw sepc, t0
    li t0, (1 << 8)          /* sstatus.SPP */
    csrs sstatus, t0

tsr_sret_insn:
    sret

tsr_after_sret:
    /* No trap occurred -> FAIL and escape to M-mode */
    la t0, suite_result
    li t1, SUITE_RESULT_FAIL
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NO_TRAP_TIMEOUT
    sw t1, 0(t0)

    /* Clear stale expectation */
    la t0, suite_expected_mode
    sw zero, 0(t0)

    /* Internal escape to M-mode via ECALL */
    la t0, suite_m_internal_escape
    li t1, 1
    sw t1, 0(t0)
    ecall

1:
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

    /* -----------------------------------------------------------------------
     * Test: SRET executes in M-mode even with TSR=1 (from tests.txt:
     * "call sret in mmode").
     * ----------------------------------------------------------------------- */
    .global smode_test_sret_in_mmode_allowed
smode_test_sret_in_mmode_allowed:
    addi sp, sp, -16
    sw ra, 12(sp)

    call suite_setup_env

    /* Any unexpected trap should return here */
    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    /* No traps expected */
    la t0, suite_expected_mode
    sw zero, 0(t0)
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

    /* Set TSR=1 (should not affect M-mode execution of sret) */
    li t0, (1 << 22)
    csrs mstatus, t0

    /* Setup SRET state: return target = sret_after, SPP=1 so we land in S-mode */
    la t0, sret_after
    csrw sepc, t0
    li t0, (1 << 8)          /* sstatus.SPP */
    csrs sstatus, t0

    /* Execute SRET in M-mode */
    sret

    /* If SRET trapped, M trap handler will mark failure and resume at 1f.
       If SRET executes, we arrive in S-mode at sret_after. */

sret_after:
    /* PASS (we reached the SRET destination, meaning SRET executed) */
    la t0, suite_result
    li t1, SUITE_RESULT_PASS
    sw t1, 0(t0)
    la t0, suite_fail_reason
    li t1, SUITE_FAIL_NONE
    sw t1, 0(t0)

    /* Escape back to M-mode so the suite can continue */
    la t0, suite_m_internal_escape
    li t1, 1
    sw t1, 0(t0)
    ecall

1:
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

    /* -----------------------------------------------------------------------
     * Test: TVM blocks SFENCE.VMA in S-mode (merged from tests.txt variants).
     * ----------------------------------------------------------------------- */
    .global smode_test_tvm_blocks_sfence_vma
smode_test_tvm_blocks_sfence_vma:
    addi sp, sp, -16
    sw ra, 12(sp)

    call suite_setup_env

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    /* Expect M-mode illegal instruction at sfence.vma */
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
    la t1, tvm_sfence_insn
    sw t1, 0(t0)

    la t0, suite_expected_tval_valid
    sw zero, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    /* Enter S-mode */
    li t0, 0x1800
    csrc mstatus, t0
    li t0, 0x800
    csrs mstatus, t0

    /* Set TVM=1 */
    li t0, (1 << 20)
    csrs mstatus, t0

    la t0, tvm_smode_entry_sfence
    csrw mepc, t0
    mret

tvm_smode_entry_sfence:
tvm_sfence_insn:
    sfence.vma x0, x0

    /* If we reach here, no trap -> FAIL and escape */
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
     * Test: TVM blocks SATP CSR access in S-mode (merged from tests.txt variants).
     * ----------------------------------------------------------------------- */
    .global smode_test_tvm_blocks_satp_access
smode_test_tvm_blocks_satp_access:
    addi sp, sp, -16
    sw ra, 12(sp)

    call suite_setup_env

    la t0, suite_resume_mepc
    la t1, 1f
    sw t1, 0(t0)

    /* Expect M-mode illegal instruction at csrw satp */
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
    la t1, tvm_satp_insn
    sw t1, 0(t0)

    la t0, suite_expected_tval_valid
    sw zero, 0(t0)

    la t0, suite_result
    sw zero, 0(t0)
    la t0, suite_fail_reason
    sw zero, 0(t0)
    la t0, suite_m_internal_escape
    sw zero, 0(t0)

    /* Enter S-mode */
    li t0, 0x1800
    csrc mstatus, t0
    li t0, 0x800
    csrs mstatus, t0

    /* Set TVM=1 */
    li t0, (1 << 20)
    csrs mstatus, t0

    la t0, tvm_smode_entry_satp
    csrw mepc, t0
    mret

tvm_smode_entry_satp:
    li t0, 0
tvm_satp_insn:
    csrw satp, t0

    /* If we reach here, no trap -> FAIL and escape */
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
