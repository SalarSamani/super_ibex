/* Super SV32 MMU comprehensive test linker script
 *
 * M-mode code runs at physical addresses (0x100000+) with MMU off.
 * S-mode code runs through the Super MMU at virtual addresses.
 *
 * Virtual address map (lin1=0):
 *
 *   lin0=0 (exp_table_0):
 *     VA 0x0000_1000  .smode_text p1    (code, RX)
 *     VA 0x0000_2000  .smode_text p2    (code, RX)
 *     VA 0x0000_3000  .smode_text p3    (code, RX)
 *     VA 0x0000_4000  .smode_rodata     (rodata, R)
 *     VA 0x0000_5000  .smode_data       (data, RW)
 *     VA 0x0000_6000  .smode_stack      (stack, RW)
 *     VA 0x0000_7000  UART+SIM_CTRL     (PA 0x20000, RW)
 *     VA 0x0000_8000  smode_test_page   (test data, RW)
 *     VA 0x0000_9000  unmapped          (T03 store fault)
 *     VA 0x0000_A000  unmapped          (T04 fetch fault)
 *     VA 0x0000_B000  unmapped          (T05 M-mode fault)
 *     VA 0x0000_C000  noexec_page       (U RW, T07/T10/T11)
 *     VA 0x0000_D000  sonly_page        (S RWX, T08)
 *     VA 0x0000_E000  rdonly_page       (U RO, T09)
 *     VA 0x0000_F000  xonly_page        (X-only, T12/T13)
 *
 *   lin0=1 (exp_table_1):
 *     VA 0x0100_1000  fault_page        (invalid PTE, T06)
 *     VA 0x0100_2000  .umode_t06        (U code, RWXA)
 *     VA 0x0100_3000  .umode_t07        (U code, RWXA)
 *     VA 0x0100_4000  .umode_t08        (U code, RWXA)
 *     VA 0x0100_5000  .umode_t09        (U code, RWXA)
 */

OUTPUT_ARCH(riscv)

MEMORY
{
    ram   : ORIGIN = 0x00100000, LENGTH = 0x50000  /* 320 kB physical RAM */
    stack : ORIGIN = 0x00150000, LENGTH = 0x8000   /* 32 kB M-mode stack  */
}

_min_stack    = 0x2000;
_stack_len    = LENGTH(stack);
_stack_start  = ORIGIN(stack) + LENGTH(stack);

_entry_point = _vectors_start + 0x80;
ENTRY(_entry_point)

tohost   = 0x20008;
fromhost = _stack_start + 0x10;

SECTIONS
{
    /* ---- M-mode sections (physical addresses, MMU off) ---- */

    .vectors :
    {
        . = ALIGN(4);
        _vectors_start = .;
        KEEP(*(.vectors))
        _vectors_end = .;
    } > ram

    .text : {
        . = ALIGN(4);
        *(.text)
        *(.text.*)
    } > ram

    .rodata : {
        . = ALIGN(4);
        *(.srodata)
        *(.srodata.*)
        *(.rodata)
        *(.rodata.*)
    } > ram

    .data : {
        . = ALIGN(4);
        *(.sdata)
        *(.sdata.*)
        *(.data)
        *(.data.*)
    } > ram

    .bss :
    {
        . = ALIGN(4);
        _bss_start = .;
        *(.sbss)
        *(.sbss.*)
        *(.bss)
        *(.bss.*)
        *(COMMON)
        _bss_end = .;
    } > ram

    /* ---- S-mode sections (VMA = virtual, LMA = physical in ram) ---- */

    /* S-mode code: VA 0x1000, may span up to 0x3FFF (3 pages) */
    .smode_text 0x1000 : AT(ALIGN(LOADADDR(.bss) + SIZEOF(.bss), 0x1000))
    {
        _smode_text_start = .;
        *(.smode_text)
        *(.smode_text.*)
        _smode_text_end = .;
    }
    _smode_text_lma = LOADADDR(.smode_text);

    /* S-mode rodata: VA 0x4000 */
    .smode_rodata 0x4000 : AT(ALIGN(LOADADDR(.smode_text) + SIZEOF(.smode_text), 0x1000))
    {
        _smode_rodata_start = .;
        *(.smode_rodata)
        *(.smode_rodata.*)
        _smode_rodata_end = .;
    }
    _smode_rodata_lma = LOADADDR(.smode_rodata);

    /* S-mode data: VA 0x5000 */
    .smode_data 0x5000 : AT(ALIGN(LOADADDR(.smode_rodata) + SIZEOF(.smode_rodata), 0x1000))
    {
        _smode_data_start = .;
        *(.smode_data)
        *(.smode_data.*)
        _smode_data_end = .;
    }
    _smode_data_lma = LOADADDR(.smode_data);

    /* S-mode stack: VA 0x6000 (1 page) */
    .smode_stack 0x6000 : AT(ALIGN(LOADADDR(.smode_data) + SIZEOF(.smode_data), 0x1000))
    {
        _smode_stack_start = .;
        . = . + 0x1000;
        _smode_stack_end = .;
    }
    _smode_stack_lma = LOADADDR(.smode_stack);

    /* S-mode test page: VA 0x8000 (RW data page for T01/T02) */
    .smode_test_page 0x8000 : AT(ALIGN(LOADADDR(.smode_stack) + SIZEOF(.smode_stack), 0x1000))
    {
        _smode_test_page_start = .;
        . = . + 0x1000;
        _smode_test_page_end = .;
    }
    _smode_test_page_lma = LOADADDR(.smode_test_page);

    /* Physical backing pages for permission tests (allocated in RAM, mapped by C code) */
    .smode_noexec_page 0xC000 : AT(ALIGN(LOADADDR(.smode_test_page) + SIZEOF(.smode_test_page), 0x1000))
    {
        _smode_noexec_page_start = .;
        . = . + 0x1000;
        _smode_noexec_page_end = .;
    }
    _smode_noexec_page_lma = LOADADDR(.smode_noexec_page);

    .smode_sonly_page 0xD000 : AT(ALIGN(LOADADDR(.smode_noexec_page) + SIZEOF(.smode_noexec_page), 0x1000))
    {
        _smode_sonly_page_start = .;
        . = . + 0x1000;
        _smode_sonly_page_end = .;
    }
    _smode_sonly_page_lma = LOADADDR(.smode_sonly_page);

    .smode_rdonly_page 0xE000 : AT(ALIGN(LOADADDR(.smode_sonly_page) + SIZEOF(.smode_sonly_page), 0x1000))
    {
        _smode_rdonly_page_start = .;
        . = . + 0x1000;
        _smode_rdonly_page_end = .;
    }
    _smode_rdonly_page_lma = LOADADDR(.smode_rdonly_page);

    .smode_xonly_page 0xF000 : AT(ALIGN(LOADADDR(.smode_rdonly_page) + SIZEOF(.smode_rdonly_page), 0x1000))
    {
        _smode_xonly_page_start = .;
        . = . + 0x1000;
        _smode_xonly_page_end = .;
    }
    _smode_xonly_page_lma = LOADADDR(.smode_xonly_page);

    /* U-mode stubs: each on its own page in lin0=1 space */
    .umode_t06 0x01002000 : AT(ALIGN(LOADADDR(.smode_xonly_page) + SIZEOF(.smode_xonly_page), 0x1000))
    {
        _umode_t06_start = .;
        *(.umode_t06)
        _umode_t06_end = .;
    }
    _umode_t06_lma = LOADADDR(.umode_t06);

    .umode_t07 0x01003000 : AT(ALIGN(LOADADDR(.umode_t06) + SIZEOF(.umode_t06), 0x1000))
    {
        _umode_t07_start = .;
        *(.umode_t07)
        _umode_t07_end = .;
    }
    _umode_t07_lma = LOADADDR(.umode_t07);

    .umode_t08 0x01004000 : AT(ALIGN(LOADADDR(.umode_t07) + SIZEOF(.umode_t07), 0x1000))
    {
        _umode_t08_start = .;
        *(.umode_t08)
        _umode_t08_end = .;
    }
    _umode_t08_lma = LOADADDR(.umode_t08);

    .umode_t09 0x01005000 : AT(ALIGN(LOADADDR(.umode_t08) + SIZEOF(.umode_t08), 0x1000))
    {
        _umode_t09_start = .;
        *(.umode_t09)
        _umode_t09_end = .;
    }
    _umode_t09_lma = LOADADDR(.umode_t09);

    /* fault_page: VA 0x01001000, backed by physical page but PTE intentionally invalid */
    .smode_fault_page 0x01001000 : AT(ALIGN(LOADADDR(.umode_t09) + SIZEOF(.umode_t09), 0x1000))
    {
        _smode_fault_page_start = .;
        . = . + 0x1000;
        _smode_fault_page_end = .;
    }
    _smode_fault_page_lma = LOADADDR(.smode_fault_page);

    /* M-mode stack */
    .stack (NOLOAD): {
        . = ALIGN(4);
        . = . + _min_stack;
        . = ALIGN(4);
        stack = .;
        _stack = .;
    } > stack
}
