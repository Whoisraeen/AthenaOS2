ENTRY(_start)
PHDRS { code PT_LOAD FLAGS(5); data PT_LOAD FLAGS(6); }
SECTIONS {
    . = 0x100000;
    __load_start = .;
    .boot : ALIGN(8) { KEEP(*(.multiboot)) *(.boot.text) } :code
    .text : ALIGN(4096) { *(.text .text.*) } :code
    .rodata : ALIGN(4096) { *(.rodata .rodata.*) } :code
    .data : ALIGN(4096) { *(.data .data.*) *(.got .got.*) } :data
    __load_end = .;
    .bss : ALIGN(4096) {
        __bss_start = .;
        *(.bss .bss.*) *(COMMON)
        __bss_end = .;
    } :data
    /DISCARD/ : { *(.eh_frame*) *(.comment*) *(.note*) }
}
