// SPDX-License-Identifier: Apache-2.0
//
// Super SV32 MMU comprehensive test suite (14 tests):
//
//  --- Address translation & basic faults (S-mode) ---
//  T01: Store/load within mapped page              -> succeeds
//  T02: Unaligned load crossing page boundary      -> succeeds
//  T03: Store to unmapped page                     -> store PF (15) -> S-mode
//  T04: Fetch from unmapped page                   -> instr PF (12) -> S-mode
//  T05: Store to unmapped page (not delegated)     -> store PF (15) -> M-mode
//
//  --- U-mode permission faults (delegated to S-mode) ---
//  T06: U-mode load  from invalid PTE              -> load PF  (13)
//  T07: U-mode fetch from no-execute page          -> instr PF (12)
//  T08: U-mode load  from S-only page              -> load PF  (13)
//  T09: U-mode store to  read-only page            -> store PF (15)
//
//  --- S-mode control bits ---
//  T10: S-mode load U-page, SUM=0                  -> load PF  (13)
//  T11: S-mode load U-page, SUM=1                  -> no fault
//  T12: S-mode load X-only page, MXR=0             -> load PF  (13)
//  T13: S-mode load X-only page, MXR=1             -> no fault
//  T14: S-mode sfence.vma with TVM=1               -> illegal instr (2) -> M-mode
//
// Virtual address map (Super SV32, all lin1=0):
//
//   lin0=0, exp_table_0:
//     VA 0x1000-0x3FFF  .smode_text        (code, RX)   3 pages
//     VA 0x4000          .smode_rodata      (rodata, R)
//     VA 0x5000          .smode_data        (data, RW)
//     VA 0x6000          .smode_stack       (stack, RW)
//     VA 0x7000          UART+SIM_CTRL      (PA 0x20000, RW)
//     VA 0x8000          smode_test_page    (test data, RW)
//     VA 0x9000          unmapped           (T03 store fault)
//     VA 0xA000          unmapped           (T04 fetch fault)
//     VA 0xB000          unmapped           (T05 M-mode fault)
//     VA 0xC000          noexec_page        (U RW no X, T07/T10/T11)
//     VA 0xD000          sonly_page         (S-only RWX, T08)
//     VA 0xE000          rdonly_page        (U RO, T09)
//     VA 0xF000          xonly_page         (X-only, T12/T13)
//
//   lin0=1, exp_table_1:
//     VA 0x01001000      fault_page         (invalid PTE, T06)
//     VA 0x01002000      umode_t06          (U code, RWXA)
//     VA 0x01003000      umode_t07          (U code, RWXA)
//     VA 0x01004000      umode_t08          (U code, RWXA)
//     VA 0x01005000      umode_t09          (U code, RWXA)

#include <stdint.h>
#include "simple_system_common.h"

#define read_csr(reg)       ({ uint32_t _v; asm volatile("csrr %0," #reg : "=r"(_v)); _v; })
#define write_csr(reg, val) asm volatile("csrw " #reg ",%0" :: "r"((uint32_t)(val)))

// PTE flags
#define PTE_V (1u<<0)
#define PTE_R (1u<<1)
#define PTE_W (1u<<2)
#define PTE_X (1u<<3)
#define PTE_U (1u<<4)
#define PTE_A (1u<<6)
#define PTE_D (1u<<7)

#define PTE_PTR    PTE_V                                     // pointer PTE (V only)
#define PTE_RX     (PTE_V|PTE_R|PTE_X|PTE_A)                // S-mode code (RX)
#define PTE_RO     (PTE_V|PTE_R|PTE_A)                      // S-mode read-only
#define PTE_RW     (PTE_V|PTE_R|PTE_W|PTE_A|PTE_D)          // S-mode read-write
#define PTE_S_LEAF (PTE_V|PTE_R|PTE_W|PTE_X|PTE_A|PTE_D)   // S-mode full (RWXAD)
#define PTE_U_LEAF (PTE_S_LEAF|PTE_U)                        // U-mode full (RWXAD)
#define PTE_U_RW   (PTE_V|PTE_R|PTE_W|PTE_U|PTE_A|PTE_D)   // U-mode RW (no X)
#define PTE_U_RO   (PTE_V|PTE_R|PTE_U|PTE_A)                // U-mode R only
#define PTE_X_ONLY (PTE_V|PTE_X|PTE_A|PTE_D)                // X-only (S-mode, no R/W)

// Linker-provided LMA symbols for S-mode sections
extern char _smode_text_lma[];
extern char _smode_rodata_lma[];
extern char _smode_data_lma[];
extern char _smode_stack_lma[];
extern char _smode_test_page_lma[];
extern char _smode_noexec_page_lma[];
extern char _smode_sonly_page_lma[];
extern char _smode_rdonly_page_lma[];
extern char _smode_xonly_page_lma[];
extern char _smode_fault_page_lma[];
extern char _umode_t06_lma[];
extern char _umode_t07_lma[];
extern char _umode_t08_lma[];
extern char _umode_t09_lma[];

// S-mode text size (to determine how many code pages)
extern char _smode_text_start[];
extern char _smode_text_end[];

// Super MMU tables: 16 entries * 4 bytes = 64 bytes each, 256B aligned
static uint32_t lin0_table[16] __attribute__((aligned(256)));
static uint32_t exp_table_0[16] __attribute__((aligned(256)));
static uint32_t exp_table_1[16] __attribute__((aligned(256)));

// ASM entry points
extern void smode_entry(void);
extern void m_trap_handler(void);

// Write to CSR L1PTE0 (address 0x3C0)
static inline void write_l1pte0(uint32_t val) {
    asm volatile("csrw 0x3C0, %0" :: "r"(val));
}

// Build a leaf PTE for a 4KB page.
// For 4KB page: paddr = {phys_page[23:4], VA[11:0]}
// So phys_page[23:4] = PA[31:12], phys_page[3:0] = 0
// PTE[31:8] = phys_page[23:0]
static uint32_t make_4kb_leaf(uint32_t pa, uint32_t flags) {
    uint32_t phys_page = (pa >> 12) << 4;  // phys_page[23:4] = PA[31:12], [3:0]=0
    return (phys_page << 8) | flags;
}

// Build a pointer PTE from a 256B-aligned table address
static uint32_t make_ptr(uint32_t table_addr) {
    return (table_addr & 0xFFFFFF00u) | PTE_PTR;
}

static void mmu_init(void) {
    uint32_t i;

    for (i = 0; i < 16; i++) {
        lin0_table[i]  = 0;
        exp_table_0[i] = 0;
        exp_table_1[i] = 0;
    }

    // --- lin0=0, exp_table_0: main S-mode pages ---

    // Determine how many code pages (each 4KB) smode_text needs
    uint32_t text_size = (uint32_t)_smode_text_end - (uint32_t)_smode_text_start;
    uint32_t text_pages = (text_size + 0xFFF) >> 12;
    if (text_pages > 3) text_pages = 3;

    // Map code pages: VA 0x1000, 0x2000, 0x3000 -> consecutive physical pages
    uint32_t text_pa = (uint32_t)_smode_text_lma;
    for (i = 0; i < text_pages; i++) {
        exp_table_0[1 + i] = make_4kb_leaf(text_pa + (i << 12), PTE_RX);
    }

    // VA 0x4000 -> rodata (R)
    exp_table_0[4] = make_4kb_leaf((uint32_t)_smode_rodata_lma, PTE_RO);

    // VA 0x5000 -> data (RW)
    exp_table_0[5] = make_4kb_leaf((uint32_t)_smode_data_lma, PTE_RW);

    // VA 0x6000 -> stack (RW)
    exp_table_0[6] = make_4kb_leaf((uint32_t)_smode_stack_lma, PTE_RW);

    // VA 0x7000 -> UART PA 0x20000 (RW)
    exp_table_0[7] = make_4kb_leaf(0x00020000u, PTE_RW);

    // VA 0x8000 -> smode_test_page (RW, for T01/T02)
    exp_table_0[8] = make_4kb_leaf((uint32_t)_smode_test_page_lma, PTE_RW);

    // VA 0x9000, 0xA000, 0xB000 -> unmapped (PTE=0) for T03/T04/T05

    // VA 0xC000 -> noexec_page (U RW, no X — for T07 and T10/T11 SUM tests)
    exp_table_0[12] = make_4kb_leaf((uint32_t)_smode_noexec_page_lma, PTE_U_RW);

    // VA 0xD000 -> sonly_page (S-only RWXAD — for T08)
    exp_table_0[13] = make_4kb_leaf((uint32_t)_smode_sonly_page_lma, PTE_S_LEAF);

    // VA 0xE000 -> rdonly_page (U RO — for T09)
    exp_table_0[14] = make_4kb_leaf((uint32_t)_smode_rdonly_page_lma, PTE_U_RO);

    // VA 0xF000 -> xonly_page (X-only, S-mode — for T12/T13 MXR test)
    exp_table_0[15] = make_4kb_leaf((uint32_t)_smode_xonly_page_lma, PTE_X_ONLY);

    // --- lin0=1, exp_table_1: fault page + U-mode stubs ---

    // VA 0x01001000 -> fault_page (PTE intentionally invalid = 0) for T06

    // VA 0x01002000 -> umode_t06 (U RWXA)
    exp_table_1[2] = make_4kb_leaf((uint32_t)_umode_t06_lma, PTE_U_LEAF);

    // VA 0x01003000 -> umode_t07 (U RWXA)
    exp_table_1[3] = make_4kb_leaf((uint32_t)_umode_t07_lma, PTE_U_LEAF);

    // VA 0x01004000 -> umode_t08 (U RWXA)
    exp_table_1[4] = make_4kb_leaf((uint32_t)_umode_t08_lma, PTE_U_LEAF);

    // VA 0x01005000 -> umode_t09 (U RWXA)
    exp_table_1[5] = make_4kb_leaf((uint32_t)_umode_t09_lma, PTE_U_LEAF);

    // Wire up the table hierarchy:
    // lin0_table[0] -> exp_table_0 (pointer PTE)
    lin0_table[0] = make_ptr((uint32_t)exp_table_0);
    // lin0_table[1] -> exp_table_1 (pointer PTE)
    lin0_table[1] = make_ptr((uint32_t)exp_table_1);

    // L1PTE[0] CSR -> lin0_table (pointer PTE)
    write_l1pte0(make_ptr((uint32_t)lin0_table));

    // Enable Super SV32 MMU (satp bit 31 = enable)
    write_csr(satp, (1u << 31));
    asm volatile("sfence.vma zero, zero");
}

int main(void) {
    pcount_enable(0);
    pcount_reset();
    pcount_enable(1);

    puts("=============================================\n");
    puts("Super SV32 MMU comprehensive test (14 tests)\n");
    puts("=============================================\n");

    // Set MPP = S-mode (01), TVM = 1 (for T14)
    uint32_t ms = read_csr(mstatus);
    ms = (ms & ~(3u << 11)) | (1u << 11);  // MPP = S-mode
    ms &= ~(1u << 7);                       // MPIE = 0
    ms |= (1u << 20);                       // TVM = 1
    write_csr(mstatus, ms);

    // Delegate page faults to S-mode (cause 12, 13, 15)
    // Cause 2 (illegal instr from TVM) stays in M-mode
    write_csr(medeleg, (1u << 12) | (1u << 13) | (1u << 15));

    // Install M-mode trap handler
    write_csr(mtvec, (uint32_t)m_trap_handler);

    mmu_init();

    // mret to S-mode entry at VIRTUAL address 0x1000
    write_csr(mepc, (uint32_t)smode_entry);
    asm volatile("mret");

    for (;;);
}
