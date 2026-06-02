/**
  * @file    rk_task.h
  * @brief   rktask 实时操作系统 - 主头文件
  * @author  Reverseking
  * @note    包含所有类型定义、配置宏和 API 声明
  *          支持 Cortex-M0+/M3/M4/M7 (编译期自动检测)
  */
#ifndef __RK_TASK_H__
#define __RK_TASK_H__

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* ======================== 架构检测 (M0+ / M3 / M4 / M7) ============ */

#if defined(__ARM_ARCH_6M__)
    /* Cortex-M0+ (M0 无 PSP 不支持) */
    #define RK_ARCH_M0PLUS     1
    #include "core_cm0plus.h"
    #define rk_lock()       __disable_irq()
    #define rk_unlock()     __enable_irq()
#elif defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
    /* Cortex-M3 / M4 / M7 */
    #define RK_ARCH_M3          1
    #include "core_cm3.h"
    #define RK_BASEPRI_VAL      0xF0
    #define rk_lock()           __set_BASEPRI(RK_BASEPRI_VAL)
    #define rk_unlock()         __set_BASEPRI(0)
#else
    #error "rktask: 仅支持 Cortex-M0+/M3/M4/M7"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 配置宏 ======================== */

#define RK_TASK_MAX         16      /* 最大任务数                       */
#define RK_EVT_MAX          8       /* 最大事件组数                     */
#define RK_SLICE_MS         5       /* 时间片大小 (SysTick 次数)         */
#define RK_TICK_MS          1       /* 每 tick 毫秒数 (1/2/5/10/...)    */
#define RK_IDLE_STK_SZ      64      /* 空闲任务栈大小 (uint32_t 个数)    */
#define RK_NAME_LEN         8       /* 任务名最大长度                   */

/* ms → tick 换算 (向上取整保证最小 1 tick) */
#define RK_MS_TO_TICKS(ms)  ((ms) + RK_TICK_MS - 1) / RK_TICK_MS

/* 任务类型 */
#define RK_TYPE_COOP        0       /* 协作式任务 (需主动让出)          */
#define RK_TYPE_PREEMPT     1       /* 分时轮转任务 (时间片到期自动切换) */

/* 事件等待模式 */
#define RK_EVENT_AND        0       /* 等待全部位同时满足               */
#define RK_EVENT_OR         1       /* 等待任一位满足                   */

/* 特殊常量 */
#define RK_WAIT_FOREVER     0xFFFFFFFF  /* 无限等待                     */
#define RK_ID_ERR           0xFF        /* 无效 ID                      */
#define RK_ID_IDLE          0xFE        /* 空闲任务 ID                   */
#define RK_STATE_DELETED    0xFF        /* 僵尸任务状态标记               */

/* ======================== 类型定义 ======================== */

typedef uint8_t rk_id_t;                /* 任务/事件 ID 类型            */

/** 任务状态枚举 */
typedef enum {
    RK_READY    = 0,                    /* 就绪                         */
    RK_RUNNING  = 1,                    /* 运行                         */
    RK_BLOCKED  = 2,                    /* 阻塞 (延时或等待事件)        */
    RK_SUSPEND  = 3,                    /* 挂起                         */
} rk_state_t;

/**
  * 任务控制块 (TCB)
  * 注意: sp 必须是第一个成员 (PendSV 汇编通过偏移 0 访问);
  *       _Static_assert 强制执行此约束.
  */
typedef struct rk_task {
    uint32_t *volatile sp;              /* PSP 存档指针                  */
    uint32_t          *stack_base;      /* 栈底地址                     */

    uint32_t  stack_size;               /* 栈大小 (uint32_t 个数)       */
    uint8_t   type;                     /* RK_TYPE_COOP / PREEMPT       */
    uint8_t   state;                    /* 任务状态                     */
    uint8_t   slice_remain;             /* 时间片剩余 (仅 PREEMPT 有效)  */
    uint8_t   id;                       /* 任务 ID                      */

    uint32_t  delay_remain;             /* 延时剩余 tick                */
    uint32_t  evt_mask;                 /* 事件等待掩码                 */
    uint8_t   evt_mode;                 /* 事件等待模式 AND/OR          */
    uint8_t   evt_id;                   /* 等待的事件组 ID              */

    struct rk_task *next;               /* 链表节点                     */
    char      name[RK_NAME_LEN];        /* 任务名                       */

} rk_task_t;

_Static_assert(offsetof(rk_task_t, sp) == 0,
               "sp must be first member of rk_task_t -- PendSV asm uses offset 0");

/* ======================== API 声明 ======================== */

/* ----- 系统控制 ----- */
void rk_task_init(void);
void rk_task_start(void);

/* ----- 任务管理 ----- */
rk_id_t rk_task_create(const char *name, void (*fn)(void),
                       uint32_t *stk, uint32_t sz, uint8_t type);
void    rk_task_del(rk_id_t id);
void    rk_task_delay(uint32_t ms);
void    rk_task_yield(void);
rk_id_t rk_task_self(void);

/* ----- 事件标志 ----- */
rk_id_t rk_task_evt_new(void);
void    rk_task_evt_post(rk_id_t id, uint32_t bits);
uint32_t rk_task_evt_wait(rk_id_t id, uint32_t bits,
                          uint8_t mode, uint32_t ms);
void    rk_task_evt_del(rk_id_t id);

/* ----- 信息查询 ----- */
uint32_t rk_task_get_tick(void);
uint8_t  rk_task_get_cnt(void);
uint8_t  rk_task_stk_check(rk_id_t id);

/* ----- 调试断言 ----- */
#ifdef RK_DEBUG
#define RK_ASSERT(expr) do { \
    if (!(expr)) { __asm volatile("bkpt #0"); } \
} while (0)
#else
#define RK_ASSERT(expr) ((void)0)
#endif

/* ----- 临界区保护 ----- */
/**
  * M0+: PRIMASK (__disable_irq / __enable_irq)
  * M3+: BASEPRI (__set_BASEPRI) — 保留高优先级 ISR 响应
  *       注意: 优先级高于 BASEPRI 的 ISR 不得调用内核链表 API
  */
#ifdef __cplusplus
}
#endif

#endif /* __RK_TASK_H__ */
