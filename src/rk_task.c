/**
  * @file    rk_task.c
  * @brief   rktask 内核实现 (Cortex-M0+/M3/M4/M7)
  * @author  Reverseking
  * @note    调度器 | 任务管理 | 事件标志 | SysTick | Delta 延时队列 O(1)/tick
  */
#include "rk_task.h"

/* CMSIS-Core 头文件由用户的 HAL/标准库在 rk_task.h 之前包含。
   如果编译报错 "SCB_Type" 或 "__set_BASEPRI" 未定义,
   请在 #include "rk_task.h" 之前添加你的 MCU 头文件, 例如:
     #include "stm32f1xx_hal.h"
   ____________________________________________________________ */

/* ======================== 内核常量 ======================== */

/* ARMv7-M 架构常量 */
#define ARM_XPSR_THUMB              0x01000000U
#define ARM_EXC_RETURN_THREAD_PSP   0xFFFFFFFDU

/* 栈初始化调试标记 (传入 R3/R2/R1 用于辨识任务) */
#define RK_STK_R3_INIT              1U
#define RK_STK_R2_INIT              2U
#define RK_STK_R1_INIT              3U

/* 最小栈大小 (uint32_t 个数, 必须容纳异常帧 + 8 个通用寄存器) */
#define RK_MIN_STACK_WORDS          32U

/* SCB ICSR 寄存器位定义 */
#define SCB_ICSR_PENDSVSET          (1UL << 28)

/* SysTick 周期 = RK_TICK_MS 毫秒 (由 rk_task.h 配置) */

/* COOP 任务最大连续执行 ticks (0 = 不限, P1-10 预留) */
#define RK_COOP_MAX_TICKS           0U

/* ======================== 外部引用 ======================== */

extern uint32_t SystemCoreClock;        /* 由 HAL/启动文件提供          */

/* ======================== 链表操作 ======================== */

/** 将节点加入链表尾部 */
static void rk_list_add(rk_task_t **head,
                        rk_task_t **tail,
                        rk_task_t *node)
{
    node->next = NULL;
    if (*head == NULL)
    {
        *head = node;
        *tail = node;
    }
    else
    {
        (*tail)->next = node;
        *tail = node;
    }
}

/** 从链表头部移除并返回节点, 链表空时返回 NULL */
static rk_task_t *rk_list_pop(rk_task_t **head,
                              rk_task_t **tail)
{
    rk_task_t *node = *head;
    if (node)
    {
        *head = node->next;
        node->next = NULL;
        if (*head == NULL)
        {
            *tail = NULL;
        }
    }
    return node;
}

/** 从链表中移除指定节点 (通过地址比较)
  * @return 1=找到并移除, 0=未找到 */
static int rk_list_rem(rk_task_t **head,
                       rk_task_t **tail,
                       rk_task_t *target)
{
    rk_task_t *prev = NULL;
    rk_task_t *cur  = *head;

    while (cur)
    {
        if (cur == target)
        {
            if (prev)
            {
                prev->next = cur->next;
            }
            else
            {
                *head = cur->next;
            }

            if (*tail == cur)
            {
                *tail = prev;
            }

            cur->next = NULL;
            return 1;
        }
        prev = cur;
        cur  = cur->next;
    }
    return 0;
}

/* ======================== Delta 延时队列 ======================== */

/**
  * 按绝对剩余 tick 升序插入延时链表, 存储 delta (相对前驱的差值)
  *
  * Delta 编码使得 SysTick 只需递减头节点 (O(1)/tick),
  * 插入时的 O(N) 遍历发生在 API 调用频率 (~10-100Hz), 远低于 tick (1000Hz)
  */
static void rk_task_delay_insert(rk_task_t *t)
{
    uint32_t  remain = t->delay_remain;   /* 绝对剩余 tick 数 */
    rk_task_t *prev  = NULL;
    rk_task_t *cur   = delay_head;
    uint32_t  accum  = 0;

    /* 遍历找到排序位置 (按绝对时间升序) */
    while (cur && (accum + cur->delay_remain <= remain))
    {
        accum += cur->delay_remain;
        prev   = cur;
        cur    = cur->next;
    }

    /* t 的 delta = 剩余绝对时间 - 已累积的前驱绝对时间 */
    t->delay_remain = remain - accum;
    t->next         = cur;

    if (prev == NULL)
    {
        delay_head = t;
    }
    else
    {
        prev->next = t;
    }

    if (cur == NULL)
    {
        delay_tail = t;
    }

    /* 如果 t 后面还有节点, 从下一个节点的 delta 中减去 t 的 delta */
    if (cur)
    {
        cur->delay_remain -= t->delay_remain;
    }
}

/**
  * 从延时链表中移除任务, 将剩余 delta 传播给后继节点
  * 调用前必须确认任务确实在延时链表中
  */
static void rk_task_delay_remove(rk_task_t *t)
{
    /* 将 t 的剩余 delta 传播给后继, 保持后续节点的绝对时间不变 */
    if (t->next)
    {
        t->next->delay_remain += t->delay_remain;
    }
    rk_list_rem(&delay_head, &delay_tail, t);
}

/* ======================== 就绪链表辅助 ======================== */

/** 按任务类型加入对应就绪链表尾部 */
static void ready_add(rk_task_t *t)
{
    rk_list_add(&ready_head[t->type], &ready_tail[t->type], t);
}

/** 按任务类型从对应就绪链表中移除 */
static int ready_rem(rk_task_t *t)
{
    return rk_list_rem(&ready_head[t->type], &ready_tail[t->type], t);
}

/* ======================== 栈初始化 ======================== */

/**
  * 初始化任务栈, 模拟异常入栈后的帧布局
  *
  * 栈布局 (从高地址到低地址):
  *   [xPSR]   = 0x01000000  (bit24 Thumb 状态)
  *   [PC]     = entry       任务入口地址
  *   [LR]     = 0xFFFFFFFD  (EXC_RETURN: Thread+PSP+非特权)
  *   [R12]    = 0
  *   [R3-R1]  = 0
  *   [R0]     = arg         入口参数
  *   [R4-R11] = 0           通用寄存器
  *
  * @param  base  栈底指针 (低地址)
  * @param  sz    栈大小 (uint32_t 个数)
  * @param  entry 任务入口函数
  * @param  arg   入口参数 (传给 R0)
  * @return 初始 PSP 指针 (高地址 - 16 words)
  */
static uint32_t *rk_stk_init(uint32_t *base, uint32_t sz,
                              void *entry, uint32_t arg)
{
    uint32_t *sp = base + sz;           /* 从栈顶 (高地址) 向下压 */

    /* 在栈底放置 canary, 用于 rk_task_stk_check() 检测溢出 */
    base[0] = 0xDEADBEEFU;

    *(--sp) = ARM_XPSR_THUMB;           /* xPSR: 置位 Thumb 位       */
    *(--sp) = (uint32_t)entry;          /* PC: 任务入口地址           */
    *(--sp) = ARM_EXC_RETURN_THREAD_PSP;/* LR: EXC_RETURN 值          */
    *(--sp) = 0;                        /* R12                        */
    *(--sp) = RK_STK_R3_INIT;           /* R3 (调试标记)              */
    *(--sp) = RK_STK_R2_INIT;           /* R2 (调试标记)              */
    *(--sp) = RK_STK_R1_INIT;           /* R1 (调试标记)              */
    *(--sp) = arg;                      /* R0: 入口参数               */

    /* 按 LDMIA {R4-R11} 加载顺序: 最低地址 -> R4, 最高地址 -> R11 */
    *(--sp) = 0;                    /* R11 */
    *(--sp) = 0;                    /* R10 */
    *(--sp) = 0;                    /* R9  */
    *(--sp) = 0;                    /* R8  */
    *(--sp) = 0;                    /* R7  */
    *(--sp) = 0;                    /* R6  */
    *(--sp) = 0;                    /* R5  */
    *(--sp) = 0;                    /* R4  */

    return sp;
}

/* ======================== 内核全局状态 ===================== */

/* 任务池 */
static rk_task_t  task_pool[RK_TASK_MAX];
static uint8_t    task_used = 0;

/* 就绪链表: 按类型索引 [RK_TYPE_COOP, RK_TYPE_PREEMPT]
   rk_lock() 提供编译器屏障 (M0+: __disable_irq M3+: __set_BASEPRI);
   链表头指针无需 volatile. */
static rk_task_t *ready_head[2] = {NULL, NULL};
static rk_task_t *ready_tail[2] = {NULL, NULL};

/* 延时链表 (delta-encoded: 每个节点存储相对前驱的 tick 差值) */
static rk_task_t *delay_head   = NULL;
static rk_task_t *delay_tail   = NULL;

/* 当前运行任务 (全局, 被 rk_port.s 引用; volatile 防止编译器缓存跨 asm 的写入) */
rk_task_t *volatile rk_cur = NULL;

/* 空闲任务 (独立 TCB, 不占用任务池) */
static rk_task_t  rk_idle_tcb;
static uint32_t   rk_idle_stk[RK_IDLE_STK_SZ];

/* 系统滴答 */
volatile uint32_t rk_tick = 0;

/* 事件组池 */
static struct {
    uint32_t    flags;                  /* 当前事件状态位             */
    rk_task_t  *wait_head;              /* 等待此事件的任务链表       */
    rk_task_t  *wait_tail;
    uint8_t     used;                   /* 是否已被使用               */
} evt_pool[RK_EVT_MAX];

/* ======================== 空闲任务 ======================== */

static void rk_idle_entry(void)
{
    while (1)
    {
        __asm volatile("wfi");          /* 等待中断, 降低功耗          */
    }
}

/* ======================== 任务管理 API ==================== */

rk_id_t rk_task_create(const char *name, void (*fn)(void),
                  uint32_t *stk, uint32_t sz, uint8_t type)
{
    rk_lock();

    /* 参数校验 (在临界区内, 防止 task_used 竞态) */
    if (fn == NULL || stk == NULL)
    {
        rk_unlock();
        return RK_ID_ERR;
    }
    if (sz < RK_MIN_STACK_WORDS)
    {
        rk_unlock();
        return RK_ID_ERR;
    }
    if (type > RK_TYPE_PREEMPT)
    {
        rk_unlock();
        return RK_ID_ERR;
    }

    /* 检查栈区是否与已有任务重叠 (P1-11) */
    for (uint8_t i = 0; i < task_used; i++)
    {
        if (task_pool[i].sp != NULL)
        {
            if (!(stk + sz <= task_pool[i].stack_base
                  || task_pool[i].stack_base + task_pool[i].stack_size <= stk))
            {
                rk_unlock();
                return RK_ID_ERR;
            }
        }
    }

    /* 扫描已释放或僵尸槽位 (sp==NULL 或 state==RK_STATE_DELETED) (P0-2) */
    rk_id_t id = RK_ID_ERR;
    for (uint8_t i = 0; i < task_used; i++)
    {
        if (task_pool[i].sp == NULL || task_pool[i].state == RK_STATE_DELETED)
        {
            task_pool[i].sp = NULL;    /* 回收 zombie slot */
            id = i;
            break;
        }
    }
    if (id == RK_ID_ERR)
    {
        if (task_used >= RK_TASK_MAX)
        {
            rk_unlock();
            return RK_ID_ERR;
        }
        id = task_used++;
    }
    rk_task_t *t = &task_pool[id];

    /* 初始化 TCB */
    t->sp           = rk_stk_init(stk, sz, fn, 0);
    t->stack_base   = stk;
    t->stack_size   = sz;
    t->type         = type;
    t->state        = RK_READY;
    t->slice_remain = 0;
    t->delay_remain = 0;
    t->evt_mask     = 0;
    t->evt_mode     = 0;
    t->evt_id       = 0;
    t->id           = id;               /* P1-3: 存储自身 ID 供 O(1) rk_task_self */
    t->next         = NULL;

    if (name)
    {
        strncpy(t->name, name, RK_NAME_LEN - 1);
        t->name[RK_NAME_LEN - 1] = '\0';
    }
    else
    {
        t->name[0] = '\0';
    }

    /* 加入对应的就绪链表 */
    ready_add(t);
    rk_unlock();

    return id;
}

void rk_task_del(rk_id_t id)
{
    if (__get_IPSR() != 0)
    {
        return;   /* P1-4: 不可从 ISR 上下文调用 */
    }

    rk_lock();

    /* bounds 检查在临界区内, 避免 TOCTOU 竞态 (Issue 5) */
    if (id >= task_used)
    {
        rk_unlock();
        return;
    }
    rk_task_t *t = &task_pool[id];

    /* TOCTOU 防范: sp 检查在临界区内原子执行;
       同时跳过僵尸 slot (P0-2) */
    if (t->sp == NULL || t->state == RK_STATE_DELETED)
    {
        rk_unlock();
        return;
    }

    /* 记录是否自我删除, 以在解锁后触发切换 (Issue 19) */
    uint8_t self_del = (t == rk_cur);

    /* 从所有链表中移除 */
    ready_rem(t);

    /* 从延时链表移除: delta 编码需要传播剩余 tick 给后继 (OPT-1) */
    if (t->delay_remain > 0)
    {
        rk_task_delay_remove(t);
    }

    /* 如果任务在等待事件, 也从事件等待链表中移除 */
    if (t->evt_mask != 0 && t->evt_id < RK_EVT_MAX)
    {
        rk_list_rem(&evt_pool[t->evt_id].wait_head,
                    &evt_pool[t->evt_id].wait_tail, t);
    }

    /* P0-2: 自我删除时标记为 zombie (sp 会被 PendSV 覆盖, 不能置 NULL),
       外部删除时正常置 sp=NULL */
    if (self_del)
    {
        t->state     = RK_STATE_DELETED;   /* zombie marker */
        t->evt_mask  = 0;
        t->evt_id    = 0;
        t->evt_mode  = 0;
    }
    else
    {
        t->sp = NULL;
    }

    rk_unlock();

    /* 任务删除自己: 强制切换离开已删除的 TCB (Issue 19) */
    if (self_del)
    {
        SCB->ICSR |= SCB_ICSR_PENDSVSET;
    }
}

rk_id_t rk_task_self(void)
{
    if (rk_cur == NULL)
    {
        return RK_ID_ERR;
    }

    /* P1-3: 空闲任务返回哨兵 ID (非错误, 调用方可区分) */
    if (rk_cur == &rk_idle_tcb)
    {
        return RK_ID_IDLE;
    }

    /* P0-2: 僵尸任务 (自我删除后) 无有效 ID */
    if (rk_cur->state == RK_STATE_DELETED)
    {
        return RK_ID_ERR;
    }

    /* P1-3: O(1) 查找 — TCB 存储了自身 ID */
    return rk_cur->id;
}

/* ======================== 任务控制 ======================== */

void rk_task_delay(uint32_t ms)
{
    if (__get_IPSR() != 0)
    {
        return;   /* P1-4: 不可从 ISR 上下文调用 */
    }

    /* rk_task_delay(0) 等价于 rk_task_yield(): 不阻塞, 直接让出 CPU (Issue 23) */
    if (ms == 0)
    {
        rk_task_yield();
        return;
    }

    rk_lock();
    if (rk_cur == NULL || rk_cur == &rk_idle_tcb)
    {
        rk_unlock();
        return;
    }

    rk_cur->delay_remain = RK_MS_TO_TICKS(ms);
    rk_cur->state = RK_BLOCKED;

    /* 从就绪链表移除 */
    ready_rem(rk_cur);

    /* 按绝对时间排序插入延时链表 (delta 编码, OPT-1) */
    rk_task_delay_insert(rk_cur);

    rk_unlock();

    /* 触发 PendSV 切换到下一个就绪任务 */
    SCB->ICSR |= SCB_ICSR_PENDSVSET;
}

void rk_task_yield(void)
{
    if (__get_IPSR() != 0)
    {
        return;   /* P1-4: 不可从 ISR 上下文调用 */
    }

    rk_lock();
    if (rk_cur == NULL || rk_cur == &rk_idle_tcb)
    {
        rk_unlock();
        return;
    }

    rk_cur->state = RK_READY;

    /* 加回对应的就绪链表尾部 */
    ready_add(rk_cur);

    rk_unlock();

    /* 触发 PendSV 切换 */
    SCB->ICSR |= SCB_ICSR_PENDSVSET;
}

/* ======================== 调度器 =========================== */

/**
  * 调度函数 - 由 PendSV_Handler 或 rk_task_start() 调用
  *
  * PendSV 运行在最低优先级 (0xFF). 优先级高于 PendSV 的 ISR
  * 可能抢占 PendSV 并调用修改就绪链表的 API (如 rk_task_evt_post),
  * 导致链表损坏. 约束: PendSV 的临界区通过 BASEPRI=0xF0 保护,
  * 优先级严格高于 0xF0 (数值小于 0xF0) 的 ISR 不得调用修改
  * 内核链表的 API.
  *
  * 选择顺序: COOP 就绪 -> PREEMPT 就绪 -> 空闲任务
  * 轮转方式: 由 rk_task_yield() (COOP) 或 SysTick (PREEMPT) 将任务
  *           放回就绪链表尾部实现轮转.
  *
  * @return 新任务的 PSP 栈指针 (OPT-3: 直接传给 PendSV 的 r0)
  */
uint32_t *rk_sched(void)
{
    rk_task_t *next;

    /* 临界区: 防止 SysTick (0xF0) 在链表操作期间抢占 (Issues 1, 10) */
    rk_lock();

    /* 1) 优先选择 COOP 任务 */
    if (ready_head[RK_TYPE_COOP])
    {
        next = rk_list_pop(&ready_head[RK_TYPE_COOP],
                           &ready_tail[RK_TYPE_COOP]);
        next->state = RK_RUNNING;
        rk_cur = next;
        rk_unlock();
        return next->sp;
    }

    /* 2) 无 COOP, 选 PREEMPT 任务 */
    if (ready_head[RK_TYPE_PREEMPT])
    {
        next = rk_list_pop(&ready_head[RK_TYPE_PREEMPT],
                           &ready_tail[RK_TYPE_PREEMPT]);
        next->state       = RK_RUNNING;
        next->slice_remain = RK_SLICE_MS;
        rk_cur = next;
        rk_unlock();
        return next->sp;
    }

    /* 3) 没有任何任务就绪, 运行空闲任务 */
    rk_cur = &rk_idle_tcb;
    rk_idle_tcb.state = RK_RUNNING;         /* Issue 21: 保持 state 一致 */
    rk_unlock();
    return rk_idle_tcb.sp;
}

/* ======================== SysTick 处理 ===================== */

/**
  * SysTick 中断处理 (每 1ms 触发一次)
  *
  * M3+/M4/M7: SysTick 优先级 0xF0, BASEPRI=0xF0 可屏蔽.
  *   真正竞态: 优先级高于 0xF0 的 ISR 可能抢占 SysTick 并修改链表.
  *   约束: 优先级高于 0xF0 的 ISR 不得调用修改链表的 API.
  *   建议: 极限优化时可将链表操作推迟到 PendSV, 根除竞态.
  *
  * M0+: SysTick 优先级可配, PRIMASK 临界区屏蔽所有中断.
  *   临界区内 SysTick 不会触发, 链表操作天然安全.
  *
  * 功能:
  *   1. 处理延时队列: Delta 编码, O(1)/tick
  *   2. PREEMPT 时间片: 减计数, 到 0 则轮转
  *   3. COOP 优先: 如有 COOP 就绪且当前是 PREEMPT, 强制切换
  *   4. 空闲任务: 如有任务就绪则退出空闲
  */
void SysTick_Handler(void)
{
    rk_tick++;
    uint32_t need_switch = 0;

    /* ========== 处理延时等待任务 (Delta 编码: O(1)/tick, OPT-1) ========== */
    if (delay_head)
    {
        delay_head->delay_remain--;

        while (delay_head && delay_head->delay_remain == 0)
        {
            rk_task_t *expired = rk_list_pop(&delay_head, &delay_tail);
            /* expired->delay_remain == 0, 无需传播 delta */

            /* 如果任务在等待事件 (evt_mask != 0), 从事件等待链表移除并标记超时 */
            if (expired->evt_mask != 0 && expired->evt_id < RK_EVT_MAX)
            {
                rk_list_rem(&evt_pool[expired->evt_id].wait_head,
                            &evt_pool[expired->evt_id].wait_tail, expired);
                expired->evt_mask = 0;  /* 标记超时, rk_task_evt_wait 据此返回 0 */
            }

            /* 按类型加入就绪链表 */
            expired->state = RK_READY;
            ready_add(expired);
            need_switch = 1;
        }
    }

    /* ========== PREEMPT 时间片管理 ========== */
    if (rk_cur && rk_cur->type == RK_TYPE_PREEMPT
        && rk_cur->state == RK_RUNNING)
    {
        rk_cur->slice_remain--;

        if (rk_cur->slice_remain == 0)
        {
            /* 放回 PREEMPT 就绪链表尾部, 等下次轮转 */
            rk_cur->state = RK_READY;
            ready_add(rk_cur);
            need_switch = 1;
        }
    }

    /* ========== COOP 优先 ========== */
    /* 如果有 COOP 任务就绪且当前运行的是 PREEMPT, 将当前任务放回就绪链表并切换.
       否则 PREEMPT 任务被 rk_sched 弹出后不再重新入队, 变成孤儿 (Issue 15). */
    if (ready_head[RK_TYPE_COOP] && rk_cur
        && rk_cur->type == RK_TYPE_PREEMPT)
    {
        rk_cur->state = RK_READY;
        ready_add(rk_cur);
        need_switch = 1;
    }

    /* ========== 空闲任务唤醒 ========== */
    /* 空闲任务运行时, 如有任何任务就绪则强制切换 */
    if (rk_cur == &rk_idle_tcb
        && (ready_head[RK_TYPE_COOP] || ready_head[RK_TYPE_PREEMPT]))
    {
        need_switch = 1;
    }

    /* ========== 触发 PendSV ========== */
    if (need_switch)
    {
        SCB->ICSR |= SCB_ICSR_PENDSVSET;
    }
}

/* ======================== 事件标志 ========================= */

rk_id_t rk_task_evt_new(void)
{
    rk_lock();
    for (uint8_t i = 0; i < RK_EVT_MAX; i++)
    {
        if (evt_pool[i].used == 0)
        {
            evt_pool[i].flags     = 0;
            evt_pool[i].wait_head = NULL;
            evt_pool[i].wait_tail = NULL;
            evt_pool[i].used      = 1;
            rk_unlock();
            return i;
        }
    }
    rk_unlock();
    return RK_ID_ERR;
}

void rk_task_evt_post(rk_id_t id, uint32_t bits)
{
    if (id >= RK_EVT_MAX)
    {
        return;
    }
    if (evt_pool[id].used == 0)
    {
        return;
    }

    /* P0-1: 拒绝优先级高于 SysTick 的 ISR 调用 (会抢占 SysTick 并损坏链表)
       M0+ 使用 PRIMASK 临界区, 无此竞态 (PRIMASK 屏蔽所有中断包括 SysTick) */
#if defined(RK_ARCH_M3)
    {
        uint32_t ipsr = __get_IPSR();
        if (ipsr != 0
            && (NVIC_GetPriority((ipsr & 0x1FF) - 16) < RK_BASEPRI_VAL))
        {
            return;
        }
    }
#endif

    rk_lock();

    evt_pool[id].flags |= bits;

    /* 遍历等待链表, 唤醒条件满足的任务 */
    rk_task_t *t    = evt_pool[id].wait_head;
    uint8_t    woken = 0;

    while (t)
    {
        rk_task_t *next_t = t->next;    /* 提前保存, 因 t 可能被移走 */
        uint32_t satisfied = 0;

        if (t->evt_mode == RK_EVENT_AND)
        {
            /* 全部位同时满足 */
            satisfied = (evt_pool[id].flags & t->evt_mask) == t->evt_mask;
        }
        else
        {
            /* 任一位满足 */
            satisfied = (evt_pool[id].flags & t->evt_mask) != 0;
        }

        if (satisfied)
        {
            /* 从事件等待链表移除 */
            rk_list_rem(&evt_pool[id].wait_head,
                        &evt_pool[id].wait_tail, t);

            /* 如果任务也在延时队列中, 移除并传播 delta (OPT-1) */
            if (t->delay_remain > 0)
            {
                rk_task_delay_remove(t);
            }

            /* 加入就绪链表 */
            t->state = RK_READY;
            ready_add(t);

            woken = 1;
        }

        t = next_t;
    }

    rk_unlock();

    /* 只在确实有任务被唤醒时触发 PendSV (Issue 13) */
    if (woken)
    {
        SCB->ICSR |= SCB_ICSR_PENDSVSET;
    }
}

uint32_t rk_task_evt_wait(rk_id_t id, uint32_t bits,
                     uint8_t mode, uint32_t ms)
{
    if (__get_IPSR() != 0)
    {
        return 0;   /* P1-4: 不可从 ISR 上下文调用 */
    }

    if (id >= RK_EVT_MAX)
    {
        return 0;
    }
    if (evt_pool[id].used == 0)
    {
        return 0;
    }

    rk_lock();

    /* 先检查当前是否已经满足 */
    uint32_t satisfied = 0;
    if (mode == RK_EVENT_AND)
    {
        satisfied = (evt_pool[id].flags & bits) == bits;
    }
    else
    {
        satisfied = (evt_pool[id].flags & bits) != 0;
    }

    if (satisfied)
    {
        uint32_t f = evt_pool[id].flags;
        /* Auto-reset: AND 清除全部请求位, OR 只清除实际匹配的位 (Issue 7) */
        if (mode == RK_EVENT_AND)
        {
            evt_pool[id].flags &= ~bits;
        }
        else
        {
            evt_pool[id].flags &= ~(f & bits);
        }
        rk_unlock();
        return f;
    }

    /* 不满足且需要等待 */
    if (ms > 0 && rk_cur && rk_cur != &rk_idle_tcb)
    {
        rk_cur->evt_mask     = bits;
        rk_cur->evt_mode     = mode;
        rk_cur->evt_id       = id;
        rk_cur->state        = RK_BLOCKED;

        /* 从就绪链表移除 */
        ready_rem(rk_cur);

        /* 加入事件等待链表 */
        rk_list_add(&evt_pool[id].wait_head,
                    &evt_pool[id].wait_tail, rk_cur);

        /* 加入延时链表 (超时等待): ms→tick 换算后 delta 插入 */
        if (ms != RK_WAIT_FOREVER)
        {
            rk_cur->delay_remain = RK_MS_TO_TICKS(ms);
            rk_task_delay_insert(rk_cur);
        }
        else
        {
            /* 无限等待: 不在延时链表中 */
            rk_cur->delay_remain = 0;
        }

        rk_unlock();

        /* 触发 PendSV */
        SCB->ICSR |= SCB_ICSR_PENDSVSET;

        /* 重新被调度回来时, 判断唤醒原因并返回 */
        rk_lock();
        uint32_t f;
        if (rk_cur->evt_mask == bits
            && id < RK_EVT_MAX && evt_pool[id].used)
        {
            /* 事件被发布唤醒, 返回事件状态并 auto-reset (Issues 2, 7) */
            f = evt_pool[id].flags;
            if (mode == RK_EVENT_AND)
            {
                evt_pool[id].flags &= ~bits;
            }
            else
            {
                evt_pool[id].flags &= ~(f & bits);
            }
            rk_cur->evt_mask = 0;
            rk_cur->evt_id   = 0;       /* P1-12: 清除 stale 事件等待状态 */
            rk_cur->evt_mode = 0;
        }
        else
        {
            /* 超时唤醒: 检查是否有迟到的事件 (同一 tick 内 ISR 发布) (P1-9) */
            if (id < RK_EVT_MAX && evt_pool[id].used)
            {
                uint32_t f_late = evt_pool[id].flags;
                uint32_t late_ok = (mode == RK_EVENT_AND)
                    ? ((f_late & bits) == bits) : ((f_late & bits) != 0);
                if (late_ok)
                {
                    if (mode == RK_EVENT_AND)
                    {
                        evt_pool[id].flags &= ~bits;
                    }
                    else
                    {
                        evt_pool[id].flags &= ~(f_late & bits);
                    }
                    rk_cur->evt_mask = 0;
                    rk_cur->evt_id   = 0;
                    rk_cur->evt_mode = 0;
                    rk_unlock();
                    return f_late;
                }
            }
            /* 超时唤醒或事件组已删除 */
            f = 0;
            rk_cur->evt_mask = 0;
            rk_cur->evt_id   = 0;       /* P1-12 */
            rk_cur->evt_mode = 0;
        }
        rk_unlock();
        return f;
    }

    rk_unlock();
    return 0;                           /* 不等待, 直接返回 0 */
}

void rk_task_evt_del(rk_id_t id)
{
    if (id >= RK_EVT_MAX)
    {
        return;
    }
    if (evt_pool[id].used == 0)
    {
        return;
    }

    rk_lock();

    /* 唤醒所有等待此事件的任务 */
    rk_task_t *t = evt_pool[id].wait_head;
    while (t)
    {
        rk_task_t *next_t = t->next;

        /* 从延时队列移除 (若有), 传播 delta 给后继 (OPT-1) */
        if (t->delay_remain > 0)
        {
            rk_task_delay_remove(t);
        }

        t->state     = RK_READY;
        t->evt_mask  = 0;               /* 标记为中止, 非事件唤醒 */
        t->evt_id    = 0;               /* P1-12: 清除 stale 事件等待状态 */
        t->evt_mode  = 0;

        ready_add(t);

        t = next_t;
    }

    evt_pool[id].used      = 0;
    evt_pool[id].flags     = 0;
    evt_pool[id].wait_head = NULL;
    evt_pool[id].wait_tail = NULL;

    rk_unlock();
}

/* ======================== 信息查询 ======================== */

uint32_t rk_task_get_tick(void)
{
    return rk_tick;   /* rk_tick 已是 volatile, 无需局部复制 */
}

uint8_t rk_task_get_cnt(void)
{
    return task_used;
}

uint8_t rk_task_stk_check(rk_id_t id)
{
    if (id >= task_used)
    {
        return 0;
    }
    rk_task_t *t = &task_pool[id];
    if (t->sp == NULL)
    {
        return 0;
    }
    return (t->stack_base[0] != 0xDEADBEEFU);
}

/* ======================== 系统控制 ======================== */

void rk_task_init(void)
{
    /* 清空任务池 */
    memset(task_pool, 0, sizeof(task_pool));
    task_used = 0;

    /* 清空就绪链表 */
    ready_head[RK_TYPE_COOP]    = NULL;
    ready_tail[RK_TYPE_COOP]    = NULL;
    ready_head[RK_TYPE_PREEMPT] = NULL;
    ready_tail[RK_TYPE_PREEMPT] = NULL;

    /* 清空延时链表 */
    delay_head = NULL;
    delay_tail = NULL;

    /* 清空事件池 */
    memset(evt_pool, 0, sizeof(evt_pool));

    /* 重置滴答 */
    rk_tick = 0;
    rk_cur  = NULL;

    /* 创建空闲任务 (独立 TCB, 不在任务池中) */
    memset(&rk_idle_tcb, 0, sizeof(rk_idle_tcb));
    rk_idle_tcb.sp         = rk_stk_init(rk_idle_stk, RK_IDLE_STK_SZ,
                                         rk_idle_entry, 0);
    rk_idle_tcb.stack_base = rk_idle_stk;
    rk_idle_tcb.stack_size = RK_IDLE_STK_SZ;
    rk_idle_tcb.type       = RK_TYPE_COOP;
    rk_idle_tcb.state      = RK_READY;
    strncpy(rk_idle_tcb.name, "idle", RK_NAME_LEN - 1);
    rk_idle_tcb.name[RK_NAME_LEN - 1] = '\0';

    /* 空闲任务加入 COOP 就绪链表 (作为最后选择) */
    ready_add(&rk_idle_tcb);
}

void rk_task_start(void)
{
    /* 配置系统异常优先级 */
    NVIC_SetPriority(PendSV_IRQn,  0xFF);   /* PendSV: 最低优先级     */
    NVIC_SetPriority(SysTick_IRQn, 0xF0);   /* SysTick: 较高优先级    */

    /* 配置 SysTick 1ms 中断 */
    SysTick_Config(SystemCoreClock / (1000 / RK_TICK_MS));

    /* 在首次 PendSV 之前选择第一个任务并初始化 PSP, 避免
       PendSV_Handler 因 rk_cur == NULL 或 PSP == 0 而崩溃
       (OPT-3: rk_sched 直接返回新任务的 sp, 避免重复解引用) */
    __set_PSP((uint32_t)rk_sched());

    /* 触发 PendSV, 启动第一个任务 */
    SCB->ICSR |= SCB_ICSR_PENDSVSET;

    /* 调度器启动后永远不会回到这里 */
    while (1)
    {
    }
}
