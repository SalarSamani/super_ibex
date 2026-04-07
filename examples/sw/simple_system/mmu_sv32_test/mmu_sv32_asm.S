# SPDX-License-Identifier: Apache-2.0
#
# Unified Sv32 MMU test suite — 14 tests.
#
# T01-T05: S-mode address translation & basic fault tests
# T06-T09: U-mode permission fault tests
# T10-T14: S-mode control bit tests (SUM, MXR, TVM)

.section .text

.equ UART,          0x00020000
.equ SIM_CTRL,      0x00020008
.equ SSTATUS_SPP,   (1 << 8)
.equ SSTATUS_SUM,   (1 << 18)
.equ SSTATUS_MXR,   (1 << 19)
.equ SCAUSE_INSTR_PF, 12
.equ SCAUSE_LOAD_PF,  13
.equ SCAUSE_STORE_PF, 15
.equ MCAUSE_STORE_PF, 15
.equ MCAUSE_ILLEGAL,  2
.equ ECALL_FROM_S,     9
.equ NUM_TESTS,       14

.extern smode_test_page
.extern unmapped_fault_page
.extern unmapped_exec_page
.extern unmapped_mmode_page
.extern fault_page
.extern noexec_page
.extern sonly_page
.extern rdonly_page
.extern xonly_page

# =======================================================================
# M-mode trap handler (256B aligned for Ibex vectored mode)
# Handles: ecall-from-S (T05 setup), store PF in M-mode (T05),
#           illegal instr from TVM (T14)
# =======================================================================
    .balign 256
.global m_trap_handler
m_trap_handler:
    .option push
    .option rvc
    .rept 32
    c.j  m_trap_body
    c.j  m_trap_body
    .endr
    .option pop

m_trap_body:
    addi sp, sp, -16
    sw   ra, 12(sp)
    sw   s0,  8(sp)
    sw   s1,  4(sp)
    sw   s2,  0(sp)

    csrr s0, mcause
    csrr s1, mepc

    # ecall from S-mode (T05 setup: disable store PF delegation)
    li   t0, ECALL_FROM_S
    beq  s0, t0, m_handle_ecall

    # store page fault in M-mode (T05)
    li   t0, MCAUSE_STORE_PF
    beq  s0, t0, m_handle_store_pf

    # illegal instruction from TVM (T14)
    li   t0, MCAUSE_ILLEGAL
    beq  s0, t0, m_handle_tvm

    # unexpected
    la   a0, str_m_unexpected; call puts
    j    sim_exit

m_handle_ecall:
    # Clear medeleg[15] so next store PF goes to M-mode
    csrr s2, medeleg
    li   t0, (1 << 15)
    not  t0, t0
    and  s2, s2, t0
    csrw medeleg, s2
    # Return to ecall_resume address
    la   t0, ecall_resume
    lw   t0, 0(t0)
    csrw mepc, t0
    j    m_trap_ret

m_handle_store_pf:
    # T05: mark pass and skip faulting instruction
    la   t0, test_pass
    li   t1, 1
    sw   t1, 16(t0)           # test_pass[4] (T05)
    addi s1, s1, 4
    csrw mepc, s1
    # Restore medeleg[15] for later tests
    csrr s2, medeleg
    li   t0, (1 << 15)
    or   s2, s2, t0
    csrw medeleg, s2
    j    m_trap_ret

m_handle_tvm:
    # T14: mark pass and skip faulting instruction
    la   t0, test_pass
    li   t1, 1
    sw   t1, 52(t0)           # test_pass[13] (T14)
    addi s1, s1, 4
    csrw mepc, s1
    j    m_trap_ret

m_trap_ret:
    lw   ra, 12(sp)
    lw   s0,  8(sp)
    lw   s1,  4(sp)
    lw   s2,  0(sp)
    addi sp, sp, 16
    mret

# =======================================================================
# S-mode entry point — called via mret from main()
# =======================================================================
.global smode_entry
.balign 4096
smode_entry:
    # Install S-mode trap handler (for T03/T04)
    la   t0, s_trap_handler
    csrw stvec, t0

    # Zero all test_pass flags
    la   t0, test_pass
.set i, 0
.rept NUM_TESTS
    sw   zero, i(t0)
.set i, i+4
.endr

    # ===================================================================
    # T01: Unaligned load within one mapped page
    # ===================================================================
    la   a0, str_t01; call print_run

    li   t0, 0xDEADBEEF
    la   t1, smode_test_page
    addi t1, t1, 4           # offset 4, still inside page
    sw   t0, 0(t1)
    lw   t2, 0(t1)
    bne  t0, t2, t01_fail
    la   t0, test_pass
    li   t1, 1
    sw   t1, 0(t0)           # test_pass[0]
t01_fail:

    # ===================================================================
    # T02: Unaligned load crossing page boundary
    # ===================================================================
    la   a0, str_t02; call print_run

    li   t0, 0xCAFEBABE
    la   t1, smode_test_page
    li   t2, 0xFFD           # 3 bytes in page N, 1 in page N+1
    add  t1, t1, t2
    sw   t0, 0(t1)
    lw   t2, 0(t1)
    bne  t0, t2, t02_fail
    la   t0, test_pass
    li   t1, 1
    sw   t1, 4(t0)           # test_pass[1]
t02_fail:

    # ===================================================================
    # T03: Store to unmapped page -> store PF (15) -> S-mode handler
    # ===================================================================
    la   a0, str_t03; call print_run

    la   t0, expected_scause
    li   t1, SCAUSE_STORE_PF
    sw   t1, 0(t0)
    la   t0, cur_shandler_test
    li   t1, 2               # index 2 = T03
    sw   t1, 0(t0)
    la   t0, sret_mode
    sw   zero, 0(t0)         # 0 = skip (sepc += 4)

    la   t1, unmapped_fault_page
    li   t0, 0xBAD
    sw   t0, 0(t1)           # FAULT -> s_trap_handler

    # ===================================================================
    # T04: Fetch from unmapped page -> instr PF (12) -> S-mode handler
    # ===================================================================
    la   a0, str_t04; call print_run

    la   t0, expected_scause
    li   t1, SCAUSE_INSTR_PF
    sw   t1, 0(t0)
    la   t0, cur_shandler_test
    li   t1, 3               # index 3 = T04
    sw   t1, 0(t0)
    la   t0, sret_mode
    li   t1, 1               # 1 = jump to sret_target
    sw   t1, 0(t0)
    la   t0, sret_target
    la   t1, t04_resume
    sw   t1, 0(t0)

    la   t0, unmapped_exec_page
    jr   t0                   # FAULT -> s_trap_handler
t04_resume:

    # ===================================================================
    # T05: Store to unmapped page -> store PF (15) -> M-mode handler
    #      (must ecall to M-mode first to clear medeleg[15])
    # ===================================================================
    la   a0, str_t05; call print_run

    la   t0, ecall_resume
    la   t1, t05_resume
    sw   t1, 0(t0)
    ecall                     # -> M-mode clears medeleg[15], mret here

t05_resume:
    la   t1, unmapped_mmode_page
    li   t0, 0xBAD
    sw   t0, 0(t1)           # FAULT -> m_trap_handler (store PF)
    # M-mode handler marks test_pass[4] and restores medeleg[15]

    # ===================================================================
    # T06-T09: U-mode permission tests
    # Switch to U-mode via sret, faults come back to u_trap_handler
    # ===================================================================
    la   t0, u_trap_handler
    csrw stvec, t0

    # Zero cur_utest
    la   t0, cur_utest
    sw   zero, 0(t0)

    # ---- T06: U-mode load from invalid PTE -> load PF (13) ----
    la   a0, str_t06; call print_run

    la   t0, umode_t06
    csrw sepc, t0
    csrr t0, sstatus
    li   t1, SSTATUS_SPP
    not  t1, t1
    and  t0, t0, t1          # SPP=0 -> U-mode
    csrw sstatus, t0
    la   a0, fault_page      # a0 = target page
    sret                      # -> U-mode umode_t06

    # After all 4 U-mode tests, u_trap_handler jumps to smode_ctrl_tests

    # ===================================================================
    # T10-T14: S-mode control bit tests (SUM, MXR, TVM)
    # ===================================================================
smode_ctrl_tests:
    # Install skip-handler for S-mode fault tests
    la   t0, s_skip_handler
    csrw stvec, t0

    # ---- T10: SUM=0, S-mode load from U-page -> load PF (13) ----
    la   a0, str_t10; call print_run

    la   t0, cur_skip_test
    li   t1, 9               # index 9 = T10
    sw   t1, 0(t0)

    # Clear SUM
    csrr t0, sstatus
    li   t1, SSTATUS_SUM
    not  t1, t1
    and  t0, t0, t1
    csrw sstatus, t0

    la   t0, noexec_page
    lw   t0, 0(t0)           # FAULT -> s_skip_handler

    # ---- T11: SUM=1, S-mode load from U-page -> no fault ----
    la   a0, str_t11; call print_run

    # Set SUM=1
    csrr t0, sstatus
    li   t1, SSTATUS_SUM
    or   t0, t0, t1
    csrw sstatus, t0

    la   t0, noexec_page
    lw   t0, 0(t0)           # NO fault

    # Mark pass (we got here without faulting)
    la   t0, test_pass
    li   t1, 1
    sw   t1, 40(t0)          # test_pass[10] (T11)

    # Clear SUM
    csrr t0, sstatus
    li   t1, SSTATUS_SUM
    not  t1, t1
    and  t0, t0, t1
    csrw sstatus, t0

    # ---- T12: MXR=0, S-mode load from X-only -> load PF (13) ----
    la   a0, str_t12; call print_run

    la   t0, cur_skip_test
    li   t1, 11              # index 11 = T12
    sw   t1, 0(t0)

    # Clear MXR
    csrr t0, sstatus
    li   t1, SSTATUS_MXR
    not  t1, t1
    and  t0, t0, t1
    csrw sstatus, t0

    la   t0, xonly_page
    lw   t0, 0(t0)           # FAULT -> s_skip_handler

    # ---- T13: MXR=1, S-mode load from X-only -> no fault ----
    la   a0, str_t13; call print_run

    # Set MXR=1
    csrr t0, sstatus
    li   t1, SSTATUS_MXR
    or   t0, t0, t1
    csrw sstatus, t0

    la   t0, xonly_page
    lw   t0, 0(t0)           # NO fault

    # Mark pass
    la   t0, test_pass
    li   t1, 1
    sw   t1, 48(t0)          # test_pass[12] (T13)

    # Clear MXR
    csrr t0, sstatus
    li   t1, SSTATUS_MXR
    not  t1, t1
    and  t0, t0, t1
    csrw sstatus, t0

    # ---- T14: TVM=1, sfence.vma -> illegal instr (2) -> M-mode ----
    la   a0, str_t14; call print_run

    sfence.vma zero, zero     # TRAP -> m_trap_handler (TVM)
    # m_trap_handler marks test_pass[13]

    j    print_results

# =======================================================================
# S-mode trap handler for T03/T04 (basic fault tests)
# =======================================================================
    .balign 4
s_trap_handler:
    csrr t0, scause
    la   t1, expected_scause
    lw   t1, 0(t1)
    bne  t0, t1, unexpected_trap

    # Mark test_pass[cur_shandler_test]
    la   t0, cur_shandler_test
    lw   t2, 0(t0)
    la   t0, test_pass
    sll  t1, t2, 2
    add  t0, t0, t1
    li   t1, 1
    sw   t1, 0(t0)

    # Check sret_mode: 0=skip, 1=jump to sret_target
    la   t0, sret_mode
    lw   t0, 0(t0)
    bnez t0, 1f
    # skip: sepc += 4
    csrr t0, sepc
    addi t0, t0, 4
    csrw sepc, t0
    sret
1:  # jump to sret_target
    la   t0, sret_target
    lw   t0, 0(t0)
    csrw sepc, t0
    sret

# =======================================================================
# U-mode trap handler for T06-T09
# =======================================================================
    .balign 4
u_trap_handler:
    # Load cur_utest index
    la   t2, cur_utest
    lw   t2, 0(t2)

    # Check scause matches expected
    la   t3, umode_expected
    sll  t4, t2, 2
    add  t3, t3, t4
    lw   t3, 0(t3)
    csrr t0, scause
    bne  t0, t3, unexpected_trap

    # Mark test_pass[5 + cur_utest]
    la   t0, test_pass
    addi t1, t2, 5           # T06 = index 5
    sll  t1, t1, 2
    add  t0, t0, t1
    li   t1, 1
    sw   t1, 0(t0)

    # Advance cur_utest
    la   t0, cur_utest
    addi t2, t2, 1
    sw   t2, 0(t0)

    # If cur_utest == 4, all U-mode tests done -> S-mode ctrl tests
    li   t3, 4
    beq  t2, t3, go_smode_ctrl

    # Load next U-mode stub and page address
    la   t0, umode_stubs
    sll  t4, t2, 2
    add  t0, t0, t4
    lw   t0, 0(t0)
    csrw sepc, t0

    la   t0, umode_pages
    sll  t4, t2, 2
    add  t0, t0, t4
    lw   a0, 0(t0)

    # Print [RUN] for next test
    mv   s1, a0
    la   t0, umode_str_table
    sll  t4, t2, 2
    add  t0, t0, t4
    lw   s2, 0(t0)
    la   a0, str_run;  call puts
    mv   a0, s2;       call puts
    mv   a0, s1

    # sret to U-mode
    csrr t0, sstatus
    li   t1, SSTATUS_SPP
    not  t1, t1
    and  t0, t0, t1
    csrw sstatus, t0
    sret

go_smode_ctrl:
    la   t0, smode_ctrl_tests
    csrw sepc, t0
    csrr t0, sstatus
    li   t1, SSTATUS_SPP
    or   t0, t0, t1          # SPP=1 -> S-mode
    csrw sstatus, t0
    sret

# =======================================================================
# S-mode skip-handler for T10/T12 faults (sepc += 4)
# =======================================================================
    .balign 4
s_skip_handler:
    csrr t0, scause
    li   t1, SCAUSE_LOAD_PF
    bne  t0, t1, unexpected_trap

    # Mark test_pass[cur_skip_test]
    la   t0, cur_skip_test
    lw   t2, 0(t0)
    la   t0, test_pass
    sll  t1, t2, 2
    add  t0, t0, t1
    li   t1, 1
    sw   t1, 0(t0)

    csrr t0, sepc
    addi t0, t0, 4
    csrw sepc, t0
    sret

# =======================================================================
# Print results and exit
# =======================================================================
print_results:
    la   a0, str_divider; call puts

    la   s0, test_pass
    la   s4, str_table
    li   s2, 0
    li   s5, 0               # pass count
    li   s6, 0               # fail count

result_loop:
    li   t0, NUM_TESTS
    beq  s2, t0, print_summary

    # Load test_pass[s2]
    sll  t1, s2, 2
    add  t1, s0, t1
    lw   t1, 0(t1)
    li   t2, 1
    bne  t1, t2, result_fail

    # PASS
    la   a0, str_pass; call puts
    sll  t1, s2, 2
    add  t1, s4, t1
    lw   a0, 0(t1)
    call puts
    addi s5, s5, 1
    j    result_next

result_fail:
    la   a0, str_fail; call puts
    sll  t1, s2, 2
    add  t1, s4, t1
    lw   a0, 0(t1)
    call puts
    addi s6, s6, 1

result_next:
    addi s2, s2, 1
    j    result_loop

print_summary:
    la   a0, str_divider; call puts
    la   a0, str_pass_lbl; call puts
    mv   a0, s5; call putdec
    la   a0, str_fail_lbl; call puts
    mv   a0, s6; call putdec
    li   a0, '\n'; call putc
    j    sim_exit

unexpected_trap:
    la   a0, str_unexpected; call puts
    csrr t0, scause
    andi t0, t0, 0xF
    li   t1, 10
    blt  t0, t1, 1f
    addi t0, t0, 55
    j    2f
1:  addi t0, t0, 48
2:  li   t2, UART
    sb   t0, 0(t2)
    li   t0, '\n'
    sb   t0, 0(t2)
    j    sim_exit

sim_exit:
    li   t0, SIM_CTRL
    li   t1, 1
    sw   t1, 0(t0)
1:  wfi
    j    1b

# =======================================================================
# Output helpers
# =======================================================================
puts:
    li   t2, UART
1:  lb   t0, 0(a0)
    beqz t0, 2f
    sb   t0, 0(t2)
    addi a0, a0, 1
    j    1b
2:  ret

putc:
    li   t0, UART
    sb   a0, 0(t0)
    ret

# putdec: print a0 as 1-2 digit decimal
putdec:
    li   t0, 10
    blt  a0, t0, 1f
    # two digits
    addi sp, sp, -4
    sw   ra, 0(sp)
    mv   t3, a0
    li   t0, 10
    div  t1, t3, t0          # tens
    rem  t2, t3, t0          # ones
    addi a0, t1, '0'
    call putc
    addi a0, t2, '0'
    call putc
    lw   ra, 0(sp)
    addi sp, sp, 4
    ret
1:  addi a0, a0, '0'
    li   t0, UART
    sb   a0, 0(t0)
    ret

print_run:
    addi sp, sp, -8
    sw   ra, 4(sp)
    sw   s0, 0(sp)
    mv   s0, a0
    la   a0, str_run;  call puts
    mv   a0, s0;       call puts
    lw   ra, 4(sp)
    lw   s0, 0(sp)
    addi sp, sp, 8
    ret

# =======================================================================
# U-mode stubs — each on its own 4 KB page
# =======================================================================

    .balign 4096
.global umode_t06           # T06: lw from invalid PTE -> load PF (13)
umode_t06:
    lw   t0, 0(a0)
1:  j    1b

    .balign 4096
.global umode_t07           # T07: fetch into no-exec page -> instr PF (12)
umode_t07:
    jr   a0
1:  j    1b

    .balign 4096
.global umode_t08           # T08: lw from S-only page -> load PF (13)
umode_t08:
    lw   t0, 0(a0)
1:  j    1b

    .balign 4096
.global umode_t09           # T09: sw to read-only page -> store PF (15)
umode_t09:
    sw   zero, 0(a0)
1:  j    1b

    .balign 4096             # end padding

# =======================================================================
# Read-only data
# =======================================================================
.section .rodata
.balign 4

# Expected scause for U-mode tests (T06-T09)
umode_expected:
    .word 13    # T06: load PF
    .word 12    # T07: instr PF
    .word 13    # T08: load PF
    .word 15    # T09: store PF

# U-mode stub entry points
umode_stubs:
    .word umode_t06
    .word umode_t07
    .word umode_t08
    .word umode_t09

# U-mode target page addresses
umode_pages:
    .word fault_page
    .word noexec_page
    .word sonly_page
    .word rdonly_page

# U-mode test name strings (for printing from handler)
umode_str_table:
    .word str_t06
    .word str_t07
    .word str_t08
    .word str_t09

# All 14 test name string pointers
.balign 4
str_table:
    .word str_t01
    .word str_t02
    .word str_t03
    .word str_t04
    .word str_t05
    .word str_t06
    .word str_t07
    .word str_t08
    .word str_t09
    .word str_t10
    .word str_t11
    .word str_t12
    .word str_t13
    .word str_t14

str_run:          .asciz "[RUN ] "
str_pass:         .asciz "[PASS] "
str_fail:         .asciz "[FAIL] "
str_divider:      .asciz "------------------------------------------\n"
str_pass_lbl:     .asciz "  pass: "
str_fail_lbl:     .asciz "  fail: "
str_unexpected:   .asciz "UNEXPECTED TRAP scause="
str_m_unexpected: .asciz "UNEXPECTED M-mode TRAP\n"

str_t01: .asciz "T01: S-mode unaligned load (in-page)\n"
str_t02: .asciz "T02: S-mode unaligned load (cross-page)\n"
str_t03: .asciz "T03: S-mode store unmapped -> store PF to S (cause 15)\n"
str_t04: .asciz "T04: S-mode fetch unmapped -> instr PF to S (cause 12)\n"
str_t05: .asciz "T05: S-mode store unmapped -> store PF to M (cause 15)\n"
str_t06: .asciz "T06: U-mode load invalid PTE -> load PF (cause 13)\n"
str_t07: .asciz "T07: U-mode fetch no-exec page -> instr PF (cause 12)\n"
str_t08: .asciz "T08: U-mode load S-only page -> load PF (cause 13)\n"
str_t09: .asciz "T09: U-mode store read-only page -> store PF (cause 15)\n"
str_t10: .asciz "T10: S-mode load U-page SUM=0 -> load PF (cause 13)\n"
str_t11: .asciz "T11: S-mode load U-page SUM=1 -> no fault (pass)\n"
str_t12: .asciz "T12: S-mode load X-only MXR=0 -> load PF (cause 13)\n"
str_t13: .asciz "T13: S-mode load X-only MXR=1 -> no fault (pass)\n"
str_t14: .asciz "T14: S-mode sfence.vma TVM=1 -> illegal instr (cause 2)\n"

# =======================================================================
# Mutable data
# =======================================================================
.section .data
.balign 4
test_pass:        .fill NUM_TESTS, 4, 0   # one word per test (0=fail, 1=pass)
cur_utest:        .word 0                  # U-mode test index (0..3)
cur_shandler_test:.word 0                  # test index for s_trap_handler
cur_skip_test:    .word 0                  # test index for s_skip_handler
expected_scause:  .word 0                  # expected scause for s_trap_handler
sret_mode:        .word 0                  # 0=skip, 1=jump to sret_target
sret_target:      .word 0                  # address for fetch-fault return
ecall_resume:     .word 0                  # M-mode mret target after ecall
