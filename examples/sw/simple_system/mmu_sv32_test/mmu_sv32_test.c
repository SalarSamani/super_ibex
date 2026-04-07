// SPDX-License-Identifier: Apache-2.0
//
// Unified Sv32 MMU test suite (14 tests):
//
//  --- Address translation & basic faults (S-mode) ---
//  T01: Unaligned load within one page         -> succeeds
//  T02: Unaligned load crossing page boundary   -> succeeds
//  T03: Store to unmapped page                  -> store PF (15) -> S-mode
//  T04: Fetch from unmapped page                -> instr PF (12) -> S-mode
//  T05: Store to unmapped page (not delegated)  -> store PF (15) -> M-mode
//
//  --- U-mode permission faults (delegated to S-mode) ---
//  T06: U-mode load  from invalid PTE           -> load PF  (13)
//  T07: U-mode fetch from no-execute page       -> instr PF (12)
//  T08: U-mode load  from S-only page           -> load PF  (13)
//  T09: U-mode store to  read-only page         -> store PF (15)
//
//  --- S-mode control bits ---
//  T10: S-mode load U-page, SUM=0               -> load PF  (13)
//  T11: S-mode load U-page, SUM=1               -> no fault
//  T12: S-mode load X-only page, MXR=0          -> load PF  (13)
//  T13: S-mode load X-only page, MXR=1          -> no fault
//  T14: S-mode sfence.vma with TVM=1            -> illegal instr (2) -> M-mode

#include <stdint.h>
#include "simple_system_common.h"

#define read_csr(reg)       ({ uint32_t _v; asm volatile("csrr %0," #reg : "=r"(_v)); _v; })
#define write_csr(reg, val) asm volatile("csrw " #reg ",%0" :: "r"((uint32_t)(val)))

#define PGSIZE     4096u
#define PT_ENTRIES 1024u

#define PTE_V (1u<<0)
#define PTE_R (1u<<1)
#define PTE_W (1u<<2)
#define PTE_X (1u<<3)
#define PTE_U (1u<<4)
#define PTE_A (1u<<6)
#define PTE_D (1u<<7)

// S-mode leaf (no PTE_U -> S-only)
#define PTE_S_LEAF  (PTE_V|PTE_R|PTE_W|PTE_X|PTE_A|PTE_D)
// U-mode full leaf
#define PTE_U_LEAF  (PTE_S_LEAF|PTE_U)
// U-mode R+W (no X)
#define PTE_U_RW    (PTE_V|PTE_R|PTE_W|PTE_U|PTE_A|PTE_D)
// U-mode R only (no W)
#define PTE_U_RO    (PTE_V|PTE_R|PTE_U|PTE_A)
// X-only: executable, not readable/writable, S-mode (for MXR test)
#define PTE_X_ONLY  (PTE_V|PTE_X|PTE_A|PTE_D)

// Page tables (page-aligned)
static uint32_t root_pt[PT_ENTRIES] __attribute__((aligned(PGSIZE)));
static uint32_t l0_pt  [PT_ENTRIES] __attribute__((aligned(PGSIZE)));

// --- Pages for T01-T05 (address translation & basic faults) ---
volatile uint32_t smode_test_page   [PGSIZE/4] __attribute__((aligned(PGSIZE)));
volatile uint32_t unmapped_fault_page[PGSIZE/4] __attribute__((aligned(PGSIZE)));
volatile uint32_t unmapped_exec_page [PGSIZE/4] __attribute__((aligned(PGSIZE)));
volatile uint32_t unmapped_mmode_page[PGSIZE/4] __attribute__((aligned(PGSIZE)));

// --- Pages for T06-T09 (U-mode permission faults) ---
volatile uint32_t fault_page  [PGSIZE/4] __attribute__((aligned(PGSIZE)));
volatile uint32_t noexec_page [PGSIZE/4] __attribute__((aligned(PGSIZE)));
volatile uint32_t sonly_page  [PGSIZE/4] __attribute__((aligned(PGSIZE)));
volatile uint32_t rdonly_page [PGSIZE/4] __attribute__((aligned(PGSIZE)));

// --- Page for T12/T13 (MXR test) ---
volatile uint32_t xonly_page  [PGSIZE/4] __attribute__((aligned(PGSIZE)));

// ASM entry points
extern void smode_entry(void);
extern void m_trap_handler(void);

// U-mode stubs (each on its own page)
extern void umode_t06(void);
extern void umode_t07(void);
extern void umode_t08(void);
extern void umode_t09(void);

#define SET_PTE(sym, flags) do { \
    uint32_t _vpn = (uint32_t)(sym) >> 12; \
    l0_pt[_vpn & 0x3FF] = (_vpn << 10) | (flags); \
} while(0)

static void mmu_init(void) {
    uint32_t l0_ppn = (uint32_t)l0_pt >> 12;

    for (uint32_t i = 0; i < PT_ENTRIES; i++) {
        root_pt[i] = 0;
        l0_pt[i]   = 0;
    }

    // root_pt[0] -> l0_pt (covers low 4 MiB)
    root_pt[0] = (l0_ppn << 10) | PTE_V;

    // Identity-map every low-4MiB page as S-mode leaf
    for (uint32_t v = 0; v < PT_ENTRIES; v++)
        l0_pt[v] = (v << 10) | PTE_S_LEAF;

    // --- T03/T04/T05: punch out unmapped pages ---
    SET_PTE(unmapped_fault_page, 0);
    SET_PTE(unmapped_mmode_page, 0);

    // Fill exec page with 'ret' then remove PTE
    for (uint32_t i = 0; i < PGSIZE / sizeof(uint32_t); i++)
        unmapped_exec_page[i] = 0x00008067u;  // ret
    SET_PTE(unmapped_exec_page, 0);

    // --- T06: invalid PTE (zeroed) ---
    SET_PTE(fault_page, 0);

    // --- T07: U R+W but no X ---
    SET_PTE(noexec_page, PTE_U_RW);

    // --- T08: sonly_page stays S-only (default PTE_S_LEAF) ---

    // --- T09: U read-only ---
    SET_PTE(rdonly_page, PTE_U_RO);

    // --- T12/T13: X-only for MXR test ---
    SET_PTE(xonly_page, PTE_X_ONLY);

    // --- U-mode code stubs need PTE_U ---
    SET_PTE(umode_t06, PTE_U_LEAF);
    SET_PTE(umode_t07, PTE_U_LEAF);
    SET_PTE(umode_t08, PTE_U_LEAF);
    SET_PTE(umode_t09, PTE_U_LEAF);

    // Enable Sv32
    write_csr(satp, (1u<<31) | ((uint32_t)root_pt >> 12));
    asm volatile("sfence.vma zero,zero");
}

int main(void) {
    pcount_enable(0);
    pcount_reset();
    pcount_enable(1);

    puts("==========================================\n");
    puts("Sv32 MMU unified test suite (14 tests)\n");
    puts("==========================================\n");

    // MPP = S-mode (01)
    uint32_t ms = read_csr(mstatus);
    ms = (ms & ~(3u<<11)) | (1u<<11);
    ms &= ~(1u<<7);   // MPIE = 0
    ms |= (1u<<20);   // TVM = 1 (for T14)
    write_csr(mstatus, ms);

    // Delegate instr PF (12), load PF (13), store PF (15) to S-mode
    // Cause 2 (illegal instr from TVM) stays in M-mode
    write_csr(medeleg, (1u<<12) | (1u<<13) | (1u<<15));

    // Install M-mode trap handler
    write_csr(mtvec, (uint32_t)m_trap_handler);

    mmu_init();

    write_csr(mepc, (uint32_t)smode_entry);
    asm volatile("mret");

    for(;;);
}
