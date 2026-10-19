.section .text
.code64
.global timer_entry
timer_entry:
    push rax
    push rbx
    push rcx
    push rdx
    push rsi
    push rdi
    push rbp
    push r8
    push r9
    push r10
    push r11
    push r12
    push r13
    push r14
    push r15
    cld
    mov rbx, rsp
    and rsp, -16
    // The freestanding target uses no SIMD or floating-point registers.
    call athena_timer_interrupt
    mov rsp, rbx
    pop r15
    pop r14
    pop r13
    pop r12
    pop r11
    pop r10
    pop r9
    pop r8
    pop rbp
    pop rdi
    pop rsi
    pop rdx
    pop rcx
    pop rbx
    pop rax
    iretq

.global exception_entry
exception_entry:
    cli
    cld
    // Fatal handlers do not return, so error-code and non-error-code exceptions
    // can share this entry point. A dedicated IST stack is a later milestone.
    and rsp, -16
    call athena_exception_fatal
1:
    hlt
    jmp 1b
