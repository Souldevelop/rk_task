;/** @file    rk_port.S
; *  @brief   rktask 上下文切换 - PendSV 处理 (Cortex-M0+/M3/M4/M7)
; *  @author  Reverseking
; *  @note    M3/M4/M7: 9 条指令; M0+: 13 条指令
; *
; *  切换流程:
; *    PendSV 触发时, 硬件已自动入栈 {R0-R3, R12, LR, PC, xPSR}
; *    1. 手动保存 R4-R11 到当前任务的 PSP 栈
; *    2. 更新 TCB->sp 指向当前 PSP
; *    3. 调用 rk_sched() 选择下一个任务 (返回新 sp 在 r0)
; *    4. 从 r0 恢复 R4-R11
; *    5. 设置 PSP 后 BX LR 返回 (硬件自动出栈恢复 PC)
; */

    .syntax unified
    .thumb
    .text
    .align  2

    .global PendSV_Handler
    .global rk_cur
    .global rk_sched
    .type   PendSV_Handler, %function

/* ===== Architecture =====
   M0+:  define RK_PORT_M0PLUS in assembler flags (-DRK_PORT_M0PLUS)
         Uses stmia (no stmdb), 13 instructions
   M3+:  default path
         Uses stmdb/ldmia, 9 instructions

   Assembler invocation:
     M0+:  arm-none-eabi-gcc -c src/rk_task.S -o rk_task.o -DRK_PORT_M0PLUS
     M3+:  arm-none-eabi-gcc -c src/rk_task.S -o rk_task.o
     ============================ */

#if defined(RK_PORT_M0PLUS)
    /* ============ Cortex-M0+ ============ */
    .cpu    cortex-m0plus

PendSV_Handler:
    /* 1. 保存当前任务 R4-R11 到 PSP 栈
          M0+ 无 stmdb: 手动 sub + stmia 两段 4 寄存器 */
    mrs     r0, psp
    subs    r0, r0, #32
    stmia   r0!, {r4-r7}
    stmia   r0!, {r8-r11}
    subs    r0, r0, #32

    /* 2. TCB->sp = 当前 PSP */
    ldr     r1, =rk_cur
    ldr     r2, [r1]
    str     r0, [r2]

    /* 3. 调用 rk_sched() - 返回新任务的 sp 在 r0 */
    bl      rk_sched

    /* 4. 恢复 R4-R11, 设 PSP, 返回 */
    ldmia   r0!, {r4-r7}
    ldmia   r0!, {r8-r11}
    msr     psp, r0
    bx      lr

#else
    /* ============ Cortex-M3/M4/M7 (ARMv7-M) ============ */
    .cpu    cortex-m3

PendSV_Handler:
    /* 1. 保存当前任务 R4-R11 到 PSP 栈 */
    mrs     r0, psp
    stmdb   r0!, {r4-r11}

    /* 2. TCB->sp = 当前 PSP */
    ldr     r1, =rk_cur
    ldr     r2, [r1]
    str     r0, [r2]

    /* 3. 调用 rk_sched() - 返回新任务的 sp 在 r0 (OPT-3) */
    bl      rk_sched

    /* 4. 恢复 R4-R11, 设 PSP, 返回 */
    ldmia   r0!, {r4-r11}
    msr     psp, r0
    bx      lr

#endif

    .size PendSV_Handler, .-PendSV_Handler
    .end
