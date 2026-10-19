// Multiboot v1 address header lets QEMU load the flat image without an ELF32 shim.
.section .multiboot,"a"
.balign 4
.long 0x1badb002
.long 0x00010003
.long -(0x1badb002 + 0x00010003)
.long __load_start
.long __load_start
.long __load_end
.long __bss_end
.long _start

.section .boot.text,"ax"
.code32
.global _start
_start:
    cli
    cld
    cmp eax, 0x2badb002
    jne boot_failed
    // The Multiboot contract provides protected mode and A20, but no stack.
    mov edi, offset __bss_start
    mov ecx, offset __bss_end
    sub ecx, edi
    xor eax, eax
    rep stosb
    mov esp, offset stack_top

    mov eax, 0x80000000
    cpuid
    cmp eax, 0x80000001
    jb boot_failed
    mov eax, 0x80000001
    cpuid
    test edx, 1 << 29
    jz boot_failed

    // Identity map the first GiB using 512 2-MiB pages. No userspace yet.
    mov eax, offset pdpt
    or eax, 3
    mov dword ptr [pml4], eax
    mov eax, offset page_directory
    or eax, 3
    mov dword ptr [pdpt], eax
    xor ecx, ecx
1:
    mov eax, ecx
    shl eax, 21
    or eax, 0x83
    mov dword ptr [page_directory + ecx * 8], eax
    inc ecx
    cmp ecx, 512
    jne 1b
    mov eax, cr4
    or eax, 1 << 5
    mov cr4, eax
    mov eax, offset pml4
    mov cr3, eax
    mov ecx, 0xc0000080
    rdmsr
    or eax, 1 << 8
    wrmsr
    lgdt [gdt_pointer]
    mov eax, cr0
    or eax, 1 << 31
    mov cr0, eax
    // Immediate far jump to the 64-bit code segment.
    .byte 0xea
    .long long_mode
    .word 8

boot_failed:
    mov dx, 0xf4
    mov eax, 0x11
    out dx, eax
2:
    hlt
    jmp 2b

.code64
long_mode:
    mov ax, 16
    mov ds, ax
    mov es, ax
    mov ss, ax
    xor ax, ax
    mov fs, ax
    mov gs, ax
    lea rsp, [rip + stack_top]
    xor rbp, rbp
    call kernel_main
3:
    cli
    hlt
    jmp 3b

.balign 8
gdt:
    .quad 0
    .quad 0x00af9a000000ffff
    .quad 0x00cf92000000ffff
gdt_end:
gdt_pointer:
    .word gdt_end - gdt - 1
    .long gdt

.section .bss,"aw",@nobits
.balign 4096
pml4: .skip 4096
pdpt: .skip 4096
page_directory: .skip 4096
.balign 16
stack_bottom: .skip 131072
stack_top:
