# rktask 用户手册

> 适用版本: 1.0 · 目标架构: ARM Cortex-M0+/M3/M4/M7 · 2026-06-02

---

## 目录

1. [引入项目](#1-引入项目)
2. [配置指南](#2-配置指南)
3. [API 参考](#3-api-参考)
4. [调度器机制](#4-调度器机制)
5. [上下文切换](#5-上下文切换)
6. [事件标志机制](#6-事件标志机制)
7. [中断与临界区](#7-中断与临界区)
8. [栈管理与溢出检测](#8-栈管理与溢出检测)
9. [常用模式](#9-常用模式)
10. [移植说明](#10-移植说明)
11. [已知限制](#11-已知限制)
12. [常见问题](#12-常见问题)
13. [API 速查表](#13-api-速查表)

---

## 1. 引入项目

### 1.1 所需文件

| 文件 | 必需 | 说明 |
|:----|:----:|:-----|
| `rk_task.h` | ✅ | 包含全部 API 声明和配置宏 |
| `rk_task.c` | ✅ | 内核实现 |
| `rk_task.S` | ✅ | PendSV 上下文切换汇编 |

### 1.2 最小工程

```c
#include "rk_task.h"

static uint32_t stk_task[128];

static void my_task(void)
{
    while (1)
    {
        rk_task_delay(100);
    }
}

int main(void)
{
    rk_task_init();
    rk_task_create("task", my_task, stk_task, 128, RK_TYPE_PREEMPT);
    rk_task_start();
    return 0;
}
```

### 1.3 编译

```bash
# Cortex-M3 (STM32F1)
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -std=c11 -O2 \
    -c rk_task.c rk_task.S main.c -I.

# Cortex-M0+ (STM32G0): -mcpu=cortex-m0plus
# Cortex-M4  (STM32F4):  -mcpu=cortex-m4
# Cortex-M7  (STM32H7):  -mcpu=cortex-m7

arm-none-eabi-gcc -T link.ld *.o -o out.elf
```

> MCU 前缀 `__ARM_ARCH_` 系列宏由编译器自动定义，`rk_task.h` 据此选择架构路径。无需手动配置。

### 1.4 链接脚本要求

任务栈分配在 `.bss` 段，启动代码必须清零 `.bss`。无其他特殊要求。

---

## 2. 配置指南

在包含 `rk_task.h` **之前** 定义配置宏：

```c
#define RK_TASK_MAX   32      // 默认 16
#define RK_SLICE_MS   10      // 默认 5
#include "rk_task.h"
```

### 完整配置项

| 宏 | 默认值 | 范围 | 说明 |
|:---|:------:|:----|:-----|
| `RK_TASK_MAX` | 16 | 1~255 | 最大任务数。每任务占 32 字节 TCB RAM |
| `RK_EVT_MAX` | 8 | 1~255 | 最大事件组数。每组占 12 字节 RAM |
| `RK_SLICE_MS` | 5 | 1~255 | PREEMPT 任务时间片（SysTick 次数） |
| `RK_TICK_MS` | 1 | ÷1000 整除 | **SysTick 周期（毫秒）**：设为 10 = 每 10ms 触发一次 |
| `RK_IDLE_STK_SZ` | 64 | ≥16 | 空闲任务栈大小（uint32_t 个数） |
| `RK_NAME_LEN` | 8 | 1~255 | 任务名最大长度（含 '\0'） |
| `RK_DEBUG` | 未定义 | — | 定义后启用 `RK_ASSERT()` 运行时断言 |

---

## 3. API 参考

所有 API 以 `rk_task_` 为前缀，返回 `rk_id_t`（`uint8_t` 别名）。

**特殊常量：**

| 常量 | 值 | 含义 |
|:-----|:--:|:-----|
| `RK_ID_ERR` | 0xFF | 操作失败或无效 ID |
| `RK_ID_IDLE` | 0xFE | 空闲任务 ID |
| `RK_WAIT_FOREVER` | 0xFFFFFFFF | 无限等待 |

---

### 3.1 系统控制

#### `void rk_task_init(void)`

初始化内核。第一步调用，仅执行一次。

内部行为:
- `memset` 清零任务池、事件池、链表指针
- 创建空闲任务（优先级最低的 COOP 任务，执行 WFI 休眠）

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    rk_task_init();
    // 创建任务...
    rk_task_start();
}
```

---

#### `void rk_task_start(void)`

启动调度器，永不返回。

内部行为:
1. `NVIC_SetPriority(PendSV_IRQn, 0xFF)` — PendSV 最低优先级
2. `NVIC_SetPriority(SysTick_IRQn, 0xF0)` — SysTick 较高优先级
3. `SysTick_Config(SystemCoreClock / (1000 / RK_TICK_MS))` — 周期由 `RK_TICK_MS` 决定
4. `rk_sched()` 选择首个任务并设 PSP
5. 触发 PendSV 开始调度

---

### 3.2 任务管理

#### `rk_id_t rk_task_create(const char *name, void (*fn)(void), uint32_t *stk, uint32_t sz, uint8_t type)`

创建任务。

| 参数 | 说明 |
|:-----|:-----|
| `name` | 任务名，`RK_NAME_LEN-1` 字符，可为 `NULL` |
| `fn` | 任务入口函数，**必须包含无限循环** |
| `stk` | 栈缓冲区指针（`static uint32_t arr[n]`） |
| `sz` | 栈大小（uint32_t 个数，最小 32=128B） |
| `type` | `RK_TYPE_COOP`(0) 或 `RK_TYPE_PREEMPT`(1) |

| 返回值 | 含义 |
|:------|:-----|
| 0~`RK_TASK_MAX-1` | 任务 ID |
| `RK_ID_ERR` (0xFF) | 失败: 参数无效 / 池满 / 栈区重叠 |

```c
static uint32_t stk_adc[128];
rk_id_t id = rk_task_create("adc", adc_task, stk_adc, 128, RK_TYPE_PREEMPT);
if (id == RK_ID_ERR) error_handler();
```

**注意事项**:
- `fn` 不应该返回——如返回将跳转到栈中的 EXC_RETURN 值，行为未定义
- 最小栈 32 words = 128 字节（异常帧 16 words + 寄存器 8 words + 调用深度）
- 栈区与已有任务自动做**重叠检测**，重叠时返回失败
- COOP 和 PREEMPT 类型可通过 `RK_TYPE_COOP`/`RK_TYPE_PREEMPT` 指定

---

#### `void rk_task_del(rk_id_t id)`

删除任务（可删除自身）。

- 从就绪/延时/事件链表中移除
- **删除自身**：标记 `state=0xFF` (zombie)，触发 PendSV 切走。槽位在下次创建时回收
- **删除其他**：`sp=NULL`，槽位立即可重用
- 如果任务在延时队列或事件等待队列中，自动解除

```c
// 删除其他任务
rk_task_del(other_id);

// 删除自己
rk_task_del(rk_task_self());
```

**不可在 ISR 上下文调用。**

---

#### `void rk_task_delay(uint32_t ms)`

当前任务延指定毫秒数。任务进入 `RK_BLOCKED` 状态。

| `ms` | 行为 |
|:----|:----|
| `> 0` | 阻塞 ms 毫秒后恢复就绪 |
| `= 0` | 等价于 `rk_task_yield()`，不阻塞直接让出 CPU |

```c
void led_task(void)
{
    while (1)
    {
        GPIO_Toggle(LED);
        rk_task_delay(500);   // 500ms 后再翻转
    }
}
```

精度取决于 SysTick 周期（由 `RK_TICK_MS` 配置），实际延时为 `ms ± RK_TICK_MS` 毫秒。

**不可在 ISR 上下文调用。**

---

#### `void rk_task_yield(void)`

当前任务主动让出 CPU。将当前任务移至对应就绪链表尾部，触发 PendSV 切换。

- COOP 任务依赖此函数实现协作式调度
- PREEMPT 任务可用此函数提前放弃剩余时间片

```c
void coop_task(void)
{
    while (1)
    {
        step1();
        rk_task_yield();     // 给其他 COOP 机会
        step2();
        rk_task_yield();
    }
}
```

**不可在 ISR 上下文调用。**

---

#### `rk_id_t rk_task_self(void)`

返回当前运行任务 ID。**O(1)** 时间。

| 返回值 | 含义 |
|:-------|:-----|
| 0~`RK_TASK_MAX-1` | 当前任务 ID |
| `RK_ID_IDLE` (0xFE) | 当前是空闲任务 |
| `RK_ID_ERR` (0xFF) | 内核未初始化或在无效上下文 |

```c
rk_id_t me = rk_task_self();
```

---

### 3.3 事件标志

事件标志是 rktask 唯一的 IPC 机制。每个事件组有 32 个标志位（bit 0~30 可用）。

#### `rk_id_t rk_task_evt_new(void)`

创建事件组。

| 返回值 | 含义 |
|:-------|:-----|
| 0~`RK_EVT_MAX-1` | 事件组 ID |
| `RK_ID_ERR` | 已达上限 |

```c
rk_id_t evt = rk_task_evt_new();
```

---

#### `void rk_task_evt_post(rk_id_t id, uint32_t bits)`

发布事件：设置指定位，唤醒所有条件满足的等待任务。

| 参数 | 说明 |
|:-----|:-----|
| `id` | 事件组 ID |
| `bits` | 待设置的位掩码（bit 31 保留） |

- **ISR 可调用**（M3+ 需 ISR 优先级 ≥ 0xF0）
- 多个位可同时设置：`rk_task_evt_post(evt, BIT_A \| BIT_B)`
- 只在有任务被唤醒时才触发 PendSV，避免无谓切换

```c
void USART_IRQHandler(void)
{
    uint8_t d = USART->DR;
    g_data = d;
    rk_task_evt_post(evt_uart, EVT_RX_DONE);
}
```

---

#### `uint32_t rk_task_evt_wait(rk_id_t id, uint32_t bits, uint8_t mode, uint32_t ms)`

等待事件指定位被设置。

| 参数 | 说明 |
|:-----|:-----|
| `id` | 事件组 ID |
| `bits` | 待等待的位掩码 |
| `mode` | `RK_EVENT_AND` (0) 或 `RK_EVENT_OR` (1) |
| `ms` | 超时毫秒数，`RK_WAIT_FOREVER` 无限等待 |

| 返回值 | 含义 |
|:-------|:-----|
| 非 0 | 事件被发布唤醒，返回该时刻 events 的完整 flags |
| 0 | 超时 / 事件组已删除 / 不等待直接返回 |

**等待模式：**

```c
// AND — 全部位同时满足才唤醒
rk_task_evt_wait(evt, BIT_A | BIT_B, RK_EVENT_AND, 1000);

// OR — 任意一位满足即唤醒
rk_task_evt_wait(evt, BIT_A | BIT_B, RK_EVENT_OR, RK_WAIT_FOREVER);
```

**Auto-reset 语义**：唤醒时自动清除事件组中已匹配的位。AND 清除全部请求位，OR 只清除实际匹配到的位。

```c
// 示例：OR 等待 BIT_A
rk_task_evt_post(evt, BIT_A | BIT_B);     // post 了两个位
uint32_t f = rk_task_evt_wait(evt, BIT_A, RK_EVENT_OR, 1000);
// f 包含 BIT_A 和 BIT_B（事件组完整快照）
// 事件组: BIT_A cleared, BIT_B 保留（因 A 是匹配位，B 不是）
```

**不可在 ISR 上下文调用。**

---

#### `void rk_task_evt_del(rk_id_t id)`

删除事件组。所有等待此组任务的 `rk_task_evt_wait` 返回 0。

```c
rk_task_evt_del(evt_id);
```

---

### 3.4 信息查询

#### `uint32_t rk_task_get_tick(void)`

系统 tick 计数（自 `rk_task_init` 起毫秒数）。约 49.7 天回绕一次 (2^32 ms)。

测量间隔时安全地使用无符号减法：

```c
uint32_t t0 = rk_task_get_tick();
do_work();
uint32_t elapsed = rk_task_get_tick() - t0;   // 回绕安全
```

---

#### `uint8_t rk_task_get_cnt(void)`

当前已创建任务数（含空闲任务）。

```c
uint8_t n = rk_task_get_cnt();
```

---

#### `uint8_t rk_task_stk_check(rk_id_t id)`

检查任务栈溢出。返回非 0 表示栈底金丝雀 (0xDEADBEEF) 已被破坏。

| 返回值 | 含义 |
|:------:|:------|
| 0 | 栈完好（或 ID 无效） |
| 1 | 栈溢出 |

```c
if (rk_task_stk_check(tid))
{
    // 栈已损坏 — 增大栈或检查递归/局部数组大小
}
```

---

### 3.5 临界区宏

#### `rk_lock()` / `rk_unlock()`

进入/退出临界区。

| 架构 | 实现 | 效果 |
|:----|:----|:-----|
| M0+ | `__disable_irq()` / `__enable_irq()` | 屏蔽所有可屏蔽中断 |
| M3+ | `__set_BASEPRI(0xF0)` / `__set_BASEPRI(0)` | 屏蔽优先级 ≥ 0xF0 的中断 |

```c
rk_lock();
shared_counter++;
g_buffer[head++] = data;
rk_unlock();
```

---

### 3.6 调试断言

```c
#ifdef RK_DEBUG
#define RK_ASSERT(expr) do { \
    if (!(expr)) { __asm volatile("bkpt #0"); } \
} while (0)
#else
#define RK_ASSERT(expr) ((void)0)
#endif
```

```c
#define RK_DEBUG
#include "rk_task.h"

void task(void)
{
    RK_ASSERT(rk_task_self() != RK_ID_IDLE);   // 空任务不执行此逻辑
}
```

---

## 4. 调度器机制

### 4.1 调度决策

`rk_sched()` 在 PendSV 中被调用，按优先级选择下一个任务：

```
Step 1: COOP 就绪链表非空 → 取头节点运行
Step 2: PREEMPT 就绪链表非空 → 取头节点、重置时间片、运行
Step 3: 都为空 → 运行空闲任务（WFI 休眠）
```

选择下一个任务后，直接返回其 `sp` 值给 PendSV（由 r0 传递）。

### 4.2 调度触发时刻

| 触发点 | 原因 | 调度结果 |
|:-------|:-----|:---------|
| `rk_task_delay(n)` | 主动延时 | 当前阻塞，切其他任务 |
| `rk_task_delay(0)` | 等同 yield | 当前就绪尾，切其他 |
| `rk_task_yield()` | 主动让出 | 当前就绪尾，切其他 |
| `rk_task_evt_wait()` | 等待事件 | 当前阻塞，切其他 |
| `rk_task_evt_post()` | 唤醒等待者 | 如唤醒成功则切 |
| `rk_task_del(self)` | 删除自己 | 强制切离 |
| `SysTick_Handler` | 延时到期/时间片耗尽/COOP 优先触发 | 按规则切换 |

### 4.3 SysTick 中的调度逻辑

每个 SysTick（周期由 `RK_TICK_MS` 配置）的 `SysTick_Handler` 完成四件事：

```
1. Delta 延时队列：递减头节点 delay_remain。归零则弹出并加入就绪 (O(1))
2. PREEMPT 时间片：减 slice_remain。归零则当前任务重回就绪尾
3. COOP 优先：若 COOP 就绪且当前是 PREEMPT，将当前放回就绪尾
4. 唤醒空闲：若空闲任务运行且任何就绪链非空，触发切换
```

### 4.4 Delta 延时队列算法

#### 存储格式

延时链表中每个节点存储的是与前驱的**绝对时间差值**，而非绝对剩余值。

```
插入时（API 调用，O(N) 低频）:
  任务 T 要求延时 50ms:
    delay_head → [T1: 15] → [T2: 35] → [T3: 50]  ← 每个节点存与前驱差值
                (绝对 15)  (绝对 50)  (绝对 100)

   遍历累加: T1.delta=15, T1+T2.delta=50, T1+T2+T3.delta=100
   插入 T.delay_remain=50 在 T1 和 T2 之间:
     → [T1: 15] → [T: 35] → [T2: 0]  → [T3: 50]  ← T2.delta 被修正
                     (50-15)   (50-50)    (100-100)
   实际 T2.delta 减掉 T 所占的间隔

处理时（SysTick，O(1) 1kHz）:
  每 tick 只做: delay_head->delay_remain--  （1 条减法）
  归零时: 弹出 head，后面的节点 delta 不变（绝对时间不变）
```

#### Delta 编码的意义

| 方案 | 每 tick 操作 | 16 任务全延时时的负载 |
|:----|:------------|:-------------------|
| 遍历减值 | 16 次递减 + 16 次比较 | ~50 指令/ms |
| Delta 编码 | **1 次递减 + 1 次比较** | **~5 指令/ms** |

---

## 5. 上下文切换

### 5.1 PendSV 机制

Cortex-M 的 PendSV 异常被编程为**最低优先级**（0xFF）。当任务或 ISR 要求切换时，仅通过写 `SCB->ICSR` 的 PENDSVSET 位触发 PendSV。PendSV 在所有其他异常处理完毕后才会执行。这保证了：

- 中断服务期间不会发生上下文切换
- 多个连续中断不会相互穿插任务切换
- 临界区退出后自动执行挂起的 PendSV

### 5.2 Cortex-M3/M4/M7 (9 条指令)

```asm
PendSV_Handler:
    mrs     r0, psp            ; 1. 读取当前任务的 PSP
    stmdb   r0!, {r4-r11}      ; 2. 保存 R4-R11 (预减，批量)
    ldr     r1, =rk_cur        ; 3. r1 = &rk_cur
    ldr     r2, [r1]           ; 4. r2 = rk_cur (当前 TCB)
    str     r0, [r2]           ; 5. TCB->sp = 新的 PSP 值
    bl      rk_sched           ; 6. rk_sched() → 新 TCB->sp 返回在 r0
    ldmia   r0!, {r4-r11}      ; 7. 恢复新任务的 R4-R11
    msr     psp, r0            ; 8. PSP = 新栈顶
    bx      lr                 ; 9. 异常返回 → 硬件自动出栈
```

### 5.3 Cortex-M0+ (13 条指令)

```asm
PendSV_Handler:
    mrs     r0, psp
    subs    r0, r0, #32        ; M0+ 无 stmdb，手动 sub
    stmia   r0!, {r4-r7}       ; 只能一次存 4 个寄存器
    stmia   r0!, {r8-r11}
    subs    r0, r0, #32
    ldr     r1, =rk_cur
    ldr     r2, [r1]
    str     r0, [r2]
    bl      rk_sched
    ldmia   r0!, {r4-r7}
    ldmia   r0!, {r8-r11}
    msr     psp, r0
    bx      lr
```

### 5.4 切换流程

以 **任务 A (PREEMPT) → 任务 B (COOP)** 为例：

```
[步骤 0] 任务 A 在 CPU 上运行，PSP 指向其栈顶。
         rk_cur = &TCB_A

[步骤 1] SysTick 触发/API 调用 → 写入 PENDSVSET
         └→ PendSV 被挂起

[步骤 2] 当前异常全部结束后，PendSV 开始执行
         硬件自动入栈: {xPSR, PC, LR, R12, R3-R0} 到 PSP_A

[步骤 3] 软件保存: stmdb r0!, {r4-r11}
         此刻 PSP_A 指向完整的 A 上下文顶部

[步骤 4] TCB_A->sp = PSP_A

[步骤 5] rk_sched() 选择任务 B:
         - A 已被放到 PREEMPT 就绪尾
         - B 在 COOP 就绪头 → 弹出
         - rk_cur = &TCB_B
         - return TCB_B->sp

[步骤 6] 软件恢复: ldmia r0!, {r4-r11}
         (r0 = TCB_B->sp，指向 B 上次保存的 R4 位置)

[步骤 7] msr psp, r0; bx lr
         └→ 硬件自动出栈: 弹出 B 的 {xPSR, PC, LR, R12, R3-R0}
         └→ PC 恢复为 B 上次被切出时的下一条指令地址

[步骤 8] 任务 B 继续执行，仿佛从未被中断
```

---

## 6. 事件标志机制

### 6.1 数据结构

```c
static struct {
    uint32_t    flags;         // 32 个事件位
    rk_task_t  *wait_head;     // 等待此事件组的任务链表
    rk_task_t  *wait_tail;
    uint8_t     used;          // 此槽位是否已分配
} evt_pool[RK_EVT_MAX];
```

### 6.2 `rk_task_evt_post` 内部流程

```
1. 检查 ID 和 used 有效性
2. [仅 M3+] 检查当前 ISR 优先级 ≥ 0xF0，否则拒绝
3. evt_pool[id].flags |= bits
4. 遍历 evt_pool[id].wait_head 链:
   - 对每个等待任务，计算:
     AND 模式: (flags & mask) == mask
     OR  模式: (flags & mask) != 0
   - 满足条件 → 从事件等待链表移除
   - 如果也在延时队列中 → 移除并传播 delta
   - 加入就绪队列
5. 如有任务被唤醒 → 触发 PendSV
```

### 6.3 `rk_task_evt_wait` 内部流程

```
1. 检查 ID 和 used 有效性
2. [快速路径] 检查当前 flags 是否已满足
   → 满足 → auto-reset → 立即返回 flags
3. [等待路径] 不满足且 ms > 0:
   - 设置 TCB 的 evt_mask/evt_mode/evt_id
   - 设置 delay_remain = ms
   - 从就绪链移除 → 加入事件等待链 → 加入延时链 (除非 WAIT_FOREVER)
   - 触发 PendSV → 阻塞
4. [唤醒后] 判断原因:
   - evt_mask 未变 → 事件唤醒 → auto-reset → 返回 flags
   - evt_mask 已变 (超时被清零) → 可能是超时
     - 额外检查：当前 flags 是否已满足 (迟到事件检测)
     - 满足 → 返回 flags
     - 不满足 → 返回 0 (超时)
```

### 6.4 AND/OR 对比

| 模式 | 唤醒条件 | 清除策略 | 典型用途 |
|:----|:---------|:---------|:---------|
| `RK_EVENT_AND` | (flags & mask) == mask | 清除 mask 全部位 | 等待多个外设全部就绪 |
| `RK_EVENT_OR` | (flags & mask) != 0 | 只清除 flags & mask 实际匹配的位 | 等待任意事件到来 |

### 6.5 超时与迟到事件

`rk_task_evt_wait` 的超时通过复用 TCB 的 `delay_remain` 字段实现——等待事件的任务同时挂在延时链表中。SysTick 中到期时，如果任务的事件条件恰好满足（同一 tick 内被高优先级 ISR 设置），`rk_task_evt_wait` 会做一次最终检查并返回正确的 flags 而非 0。

---

## 7. 中断与临界区

### 7.1 两套临界区方案

```c
#if defined(RK_ARCH_M0PLUS)
    #define rk_lock()   __disable_irq()    // PRIMASK
    #define rk_unlock() __enable_irq()
#else
    #define RK_BASEPRI_VAL   0xF0
    #define rk_lock()        __set_BASEPRI(0xF0)   // BASEPRI
    #define rk_unlock()      __set_BASEPRI(0)
#endif
```

**M0+ PRIMASK**：临界区内无任何中断（包括 SysTick）响应。ISR 调用 `rk_task_evt_post` 无竞态。

**M3+ BASEPRI**：临界区内 SysTick 被屏蔽，但高优先级中断 (优先级 < 0xF0) 仍可响应。ISR 调用 `rk_task_evt_post` 时，必须检查其优先级是否高于 SysTick——若是，拒绝调用。这是有意设计：用实时响应能力换取临界区安全。

### 7.2 ISR 调用约束表

| 函数 | ISR 中可用 | 约束 |
|:----|:----------:|:-----|
| `rk_task_evt_post` | ✅ | M3+ 需要 ISR 优先级 ≥ 0xF0 |
| `rk_lock` / `rk_unlock` | ✅ | — |
| `rk_task_evt_new` | ❌ | 仅线程上下文 |
| `rk_task_evt_wait` | ❌ | 仅线程上下文 |
| `rk_task_evt_del` | ❌ | 仅线程上下文 |
| `rk_task_delay` | ❌ | 仅线程上下文 |
| `rk_task_yield` | ❌ | 仅线程上下文 |
| `rk_task_del` | ❌ | 仅线程上下文 |

### 7.3 中断优先级设计

```
ISR 优先级 (数值越小 -> 优先级越高):
  ┌ 0x00-0xEF: 普通外设中断
  │  可通过 rk_lock()/BASEPRI 屏蔽
  │  UART、TIM、DMA、EXTI 等
  │
  ├ 0xF0:      SysTick
  │  被 BASEPRI 屏蔽 (M3+)
  │  每 tick (RK_TICK_MS ms) 处理延时/时间片
  │
  └ 0xFF:      PendSV
  │  不被 BASEPRI 屏蔽 (优先级低于阈值)
  │  上下文切换 (最低优先级)
```

**约束**：优先级高于 0xF0（数值 < 0xF0）的 ISR 不得调用 `rk_task_evt_post`，因为此类 ISR 可以抢占 SysTick 并导致链表损坏。rktask 在 M3+ 上会自动检查并拒绝此类调用。

---

## 8. 栈管理与溢出检测

### 8.1 栈初始化布局

```
                              ← stack_base + stack_size (初始 PSP)
┌──────────────────────────┐
│ xPSR  = 0x01000000       │  ARM Thumb 状态位 (bit 24=1)
│ PC    = entry             │  任务入口地址
│ LR    = 0xFFFFFFFD       │  EXC_RETURN: Thread+PSP
│ R12   = 0                 │
│ R3    = 1 (RK_STK_R3_INIT)│  调试标记
│ R2    = 2 (RK_STK_R2_INIT)│  调试标记
│ R1    = 3 (RK_STK_R1_INIT)│  调试标记
│ R0    = 0                 │  入口参数 (当前保留)
├──────────────────────────┤
│ R4~R11 = 0                │  8×4 = 32 字节
├──────────────────────────┤ ← TCB->sp 初始指向这里
│         可用栈空间        │
├──────────────────────────┤
│ canary = 0xDEADBEEF      │ ← stack_base
└──────────────────────────┘
```

### 8.2 栈大小建议

| 场景 | 建议字数 | 建议字节 | 说明 |
|:----|:-------:|:--------:|:----|
| 简单循环任务 | 64 | 256 | 无深度函数调用 |
| 中等调用深度 | 128 | 512 | 含 2-3 级函数调用 |
| 复杂任务 | 256 | 1024 | 含局部数组或协议栈 |
| 最小要求 | 32 | 128 | 仅容纳异常帧+寄存器 |

### 8.3 溢出检测

`rk_task_stk_check(id)` 检查栈底 canary 值 `0xDEADBEEF` 是否完好。如果被覆盖，说明栈已溢出到相邻内存区域（可能是其他任务的 TCB 或栈）。

```c
// 定期安全检查
void watch_task(void)
{
    while (1)
    {
        for (rk_id_t i = 0; i < rk_task_get_cnt(); i++)
        {
            if (rk_task_stk_check(i))
            {
                // 停止运行或记录错误
            }
        }
        rk_task_delay(1000);
    }
}
```

---

## 9. 常用模式

### 9.1 COOP + 事件驱动

COOP 任务做事件驱动的前端处理，PREEMPT 做周期性后台任务：

```c
static rk_id_t evt_sys;

#define EVT_BTN      0x01
#define EVT_UART     0x02
#define EVT_TIMER    0x04

/* COOP — 处理交互事件 */
static void frontend_task(void)
{
    while (1)
    {
        uint32_t f = rk_task_evt_wait(evt_sys,
                       EVT_BTN | EVT_UART, RK_EVENT_OR, RK_WAIT_FOREVER);
        if (f & EVT_BTN)     handle_button();
        if (f & EVT_UART)    handle_uart();
        /* 不 yield — 等待下一事件，期间 PREEMPT 可运行 */
    }
}

/* PREEMPT — 定期采集 */
static void backend_task(void)
{
    while (1)
    {
        sample_adc();
        rk_task_delay(50);   /* 每 50ms 采集 */
    }
}
```

> `frontend_task` 在 `rk_task_evt_wait` 阻塞期间 → PREEMPT 获取 CPU  
> `frontend_task` 被事件唤醒 → 立即执行（COOP 优先）

### 9.2 ISR 发布事件

```c
static rk_id_t evt_sys;

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & (1 << 0))
    {
        rk_task_evt_post(evt_sys, 0x01);
        EXTI->PR = (1 << 0);
    }
}
```

ISR 中发布事件后，等待事件的任务在下一个调度点被唤醒。

### 9.3 超时轮询

```c
void check_task(void)
{
    while (1)
    {
        uint32_t f = rk_task_evt_wait(evt, EVT_DATA, RK_EVENT_OR, 100);
        if (f & EVT_DATA)
        {
            process_data();
        }
        else
        {
            check_health();   // 每 100ms 超时做心跳检查
        }
    }
}
```

### 9.4 AND 模式多事件同步

```c
#define EVT_SENSOR1   0x01
#define EVT_SENSOR2   0x02
#define EVT_DMA_DONE  0x04

void fusion_task(void)
{
    while (1)
    {
        uint32_t f = rk_task_evt_wait(evt_fusion,
            EVT_SENSOR1 | EVT_SENSOR2 | EVT_DMA_DONE,
            RK_EVENT_AND, 500);
        if (f == (EVT_SENSOR1 | EVT_SENSOR2 | EVT_DMA_DONE))
        {
            sensor_fusion();
        }
        else
        {
            handle_fusion_timeout();
        }
    }
}
```

### 9.5 临界区保护共享数据

```c
static uint32_t g_adc_result;

void adc_task(void)
{
    while (1)
    {
        rk_lock();
        g_adc_result = adc_read();
        rk_unlock();
        rk_task_delay(10);
    }
}

void consume_task(void)
{
    while (1)
    {
        uint32_t val;
        rk_lock();
        val = g_adc_result;
        rk_unlock();
        process(val);
        rk_task_delay(50);
    }
}
```

---

## 10. 移植说明

### 10.1 自动检测

编译器根据 `-mcpu=` 自动定义宏：

| 宏 | 对应架构 |
|:---|:---------|
| `__ARM_ARCH_6M__` | Cortex-M0+ |
| `__ARM_ARCH_7M__` | Cortex-M3 |
| `__ARM_ARCH_7EM__` | Cortex-M4/M7 |

`rk_task.h` 据此选择 CMSIS 头文件和临界区实现。

### 10.2 手动强制指定

若编译器不定义上述宏（极少见），可在 `rk_task.h` 之前手动定义：

```c
// 强制 M3 路径
#define __ARM_ARCH_7M__    1
#include "rk_task.h"
```

### 10.3 启动文件要求

- 中断向量表中 `PendSV_Handler` 必须指向 `rk_task.S` 中的符号
- 中断向量表中 `SysTick_Handler` 必须指向 `rk_task.c` 中的函数
- `.bss` 段启动时必须清零（`rk_task_init` 依赖 `memset` 的清零语义）

### 10.4 不支持的目标

- **Cortex-M0**（无 PSP 寄存器，无法支持独立任务栈）
- **非 ARM 架构**（RISC-V、x86 等）
- **ARM7TDMI**（ARMv4T 架构，无 SysTick/PendSV）

---

## 11. 已知限制

| 限制 | 说明 |
|:----|:-----|
| 无消息队列 | 任务间传数据需使用全局变量 + 事件标志 |
| 无互斥锁 | COOP 间天然互斥；PREEMPT 间共享数据需用 `rk_lock()` 临界区 |
| 无信号量 | 可用事件标志模拟计数型信号量 |
| 最大 COOP 绑定 | COOP 任务若无限循环不 yield，PREEMPT 任务受饿 |
| 固定任务池 | 任务数编译期上限（`RK_TASK_MAX`） |
| 无 MPU 保护 | 无内存隔离，任务可越界访问 |
| 无软件定时器 | 可用 PREEMPT 任务 + `rk_task_delay` 模拟 |
| `rk_task_evt_wait` 返回 0 | 超时和事件组被删除两种场景无法区分 |
| M0 不支持 | 无 PSP，不兼容 |

---

## 12. 常见问题

### Q: 任务函数可以 return 吗？

不可以。`return` 后 PC 跳到 EXC_RETURN (0xFFFFFFFD)，触发异常或 HardFault。任务必须使用 `while(1)` 永久循环。

### Q: 为什么 PREEMPT 任务不运行？

检查是否有 COOP 任务就绪并且没有阻塞/延时。COOP 优先调度，只要有 COOP 就绪，PREEMPT 得不到 CPU。

### Q: `rk_task_delay(0)` 和 `rk_task_yield()` 有区别吗？

行为相同——都让出当前时间片。`rk_task_delay(0)` 在内部直接调用 `rk_task_yield()`。

### Q: 栈溢出后能恢复吗？

不能。溢出已破坏了相邻内存，继续运行不安全。检测到溢出后应停止运行复位系统。

### Q: 事件发布后多久能唤醒等待任务？

如果发布在 ISR 中：ISR 返回后 PendSV 执行时切换（M3+ 上几微秒内）。  
如果发布在线程中：立即进入 PendSV，执行上下文切换。

### Q: 同时等待多个事件组？

当前不支持。一个任务一次只能等待一个事件组。需要复合条件时用 AND/OR 模式。

### Q: `rk_task_evt_wait` 返回 0，如何区分超时和事件组被删除？

无法区分，两种场景都返回 0。建议在事件组删除前确保没有任务在等待，或在架构层面避免动态删除被等待中的事件组。

---

## 13. API 速查表

### 系统控制

| API | 参数 | 返回值 | ISR 安全 |
|:----|:-----|:-------|:--------|
| `rk_task_init` | — | void | ❌ |
| `rk_task_start` | — | void | ❌ |

### 任务管理

| API | 参数 | 返回值 | ISR 安全 |
|:----|:-----|:-------|:--------|
| `rk_task_create` | name, fn, stk, sz, type | `rk_id_t` | ❌ |
| `rk_task_del` | id | void | ❌ |
| `rk_task_delay` | ms | void | ❌ |
| `rk_task_yield` | — | void | ❌ |
| `rk_task_self` | — | `rk_id_t` | ❌ |

### 事件标志

| API | 参数 | 返回值 | ISR 安全 |
|:----|:-----|:-------|:--------|
| `rk_task_evt_new` | — | `rk_id_t` | ❌ |
| `rk_task_evt_post` | id, bits | void | ✅ |
| `rk_task_evt_wait` | id, bits, mode, ms | `uint32_t` | ❌ |
| `rk_task_evt_del` | id | void | ❌ |

### 信息查询

| API | 参数 | 返回值 | ISR 安全 |
|:----|:-----|:-------|:--------|
| `rk_task_get_tick` | — | `uint32_t` | ❌ |
| `rk_task_get_cnt` | — | `uint8_t` | ❌ |
| `rk_task_stk_check` | id | `uint8_t` | ❌ |

### 宏

| 宏 | 参数 | 说明 |
|:---|:-----|:-----|
| `rk_lock()` | — | 进入临界区 |
| `rk_unlock()` | — | 退出临界区 |
| `RK_ASSERT(expr)` | 条件表达式 | 调试断言 (`#ifdef RK_DEBUG`) |

### 任务类型

| 常量 | 值 | 说明 |
|:-----|:--:|:-----|
| `RK_TYPE_COOP` | 0 | 协作式（不抢占，需 yield） |
| `RK_TYPE_PREEMPT` | 1 | 抢占式（时间片轮转） |

### 事件模式

| 常量 | 值 | 说明 |
|:-----|:--:|:-----|
| `RK_EVENT_AND` | 0 | 等待所有位同时满足 |
| `RK_EVENT_OR` | 1 | 等待任一位满足 |

---

> © 2026 Reverseking. MIT License.
