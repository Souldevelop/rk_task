/**
  * @file    rk_key_filter.h
  * @brief   rktask 按键滤波 & 旋转编码器解码组件
  * @author  Reverseking
  * @note    纯 C 实现, 无外部依赖, 支持任意数量实例
  *
  * 按键滤波:
  *   连续 N 次读到相同电平 → 确认按键
  *   支持按下/释放/长按事件, 通过事件标志通知
  *
  * 编码器解码:
  *   AB 相位查表法, 硬件去抖后直接输入电平
  *   支持任意数量编码器, 各自独立实例
  */
#ifndef __RK_KEY_FILTER_H__
#define __RK_KEY_FILTER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 配置宏 ======================== */

#define RK_BTN_SCAN_MS      5       /* 默认扫描周期 (ms)               */
#define RK_BTN_THRESHOLD     4       /* 默认滤波阈值 (连续相同次数)     */

/* ======================== 事件位封装 ======================== */

/** 按键事件位: 每个按键占用 3 个连续 bit */
#define RK_BTN_DOWN(n)      (1UL << ((n) * 3))        /* 按下   */
#define RK_BTN_UP(n)        (1UL << ((n) * 3 + 1))    /* 释放   */
#define RK_BTN_LONG(n)      (1UL << ((n) * 3 + 2))    /* 长按   */

/** 编码器事件位: 每个编码器占用 2 个连续 bit */
#define RK_ENC_CW(n)        (1UL << ((n) * 2))         /* 正转   */
#define RK_ENC_CCW(n)       (1UL << ((n) * 2 + 1))     /* 反转   */

/* ======================== 按键滤波 ======================== */

/** 按键滤波实例 (4 字节) */
typedef struct {
    uint8_t   count;                /* 连续相同计数                    */
    uint8_t   threshold;            /* 滤波阈值 (典型 3~5)            */
} rk_btn_t;

/**
  * @brief  初始化按键滤波实例
  * @param  btn       按键实例指针
  * @param  threshold 滤波阈值 (连续相同次数, 0=直通)
  */
static inline void rk_btn_init(rk_btn_t *btn, uint8_t threshold)
{
    btn->count     = 0;
    btn->threshold = (threshold == 0) ? 1 : threshold;
}

/**
  * @brief  扫描按键, 返回稳定后的键值
  * @param  btn      按键实例指针
  * @param  active   当前 GPIO 电平 (0=按下, 1=释放, 也可用其他值)
  * @return 滤波后的键值: 0=释放, 非0=按下 (返回 threshold 次连续读到的值)
  *
  * 调用者需定期调用 (建议 5~10ms), 返回值变化表示按键状态已确认.
  * 配合事件标志使用:
  *   if (上次释放 && 本次按下) → rk_task_evt_post(evt, RK_BTN_DOWN(n));
  *   if (上次按下 && 本次释放) → rk_task_evt_post(evt, RK_BTN_UP(n));
  */
static inline uint32_t rk_btn_scan(rk_btn_t *btn, uint32_t active)
{
    if (active == btn->count >> 7)    /* 高阶位存上次确认值 */
    {
        /* 状态不变 → 递增计数 */
        if (btn->count < (btn->threshold | 0x80))
        {
            btn->count++;
        }
    }
    else
    {
        /* 状态变化 → 重置计数, 记录新状态 */
        btn->count = 0x80 | active;
    }

    /* 返回当前确认值: 高阶位 (bit7) 为 1 时表示已稳定 */
    return (btn->count & 0x80) ? (btn->count & 1) : btn->count >> 7;
}

/* ======================== 旋转编码器 ======================== */

/** 编码器解码实例 (3 字节) */
typedef struct {
    uint8_t   state;                /* 当前相位 (0~3)                 */
    int16_t   pulse;                /* 累积脉冲数                     */
} rk_encoder_t;

/**
  * @brief  相位查表: 索引 = (prev_state << 2) | curr_state
  *         值:  0=无变化, +1=CW, -1=CCW
  */
static const int8_t rk_enc_table[16] = {
     0,  1, -1,  0,    /* 00→00, 00→01, 00→10, 00→11 */
    -1,  0,  0,  1,    /* 01→00, 01→01, 01→10, 01→11 */
     1,  0,  0, -1,    /* 10→00, 10→01, 10→10, 10→11 */
     0, -1,  1,  0     /* 11→00, 11→01, 11→10, 11→11 */
};

/**
  * @brief  初始化编码器实例
  * @param  enc  编码器实例指针
  */
static inline void rk_encoder_init(rk_encoder_t *enc)
{
    enc->state = 0;
    enc->pulse = 0;
}

/**
  * @brief  扫描编码器 AB 相位, 解码方向
  * @param  enc  编码器实例指针
  * @param  a    A 相当前电平 (0/1)
  * @param  b    B 相当前电平 (0/1)
  * @return +1 = 正转, -1 = 反转, 0 = 无变化或无效跳变
  *
  * 相位查表解码, 支持单步/倍速.
  * 脉冲计数累积在 enc->pulse, 可随时读取或清零.
  */
static inline int rk_encoder_scan(rk_encoder_t *enc, uint8_t a, uint8_t b)
{
    uint8_t curr = (a << 1) | b;                /* 当前 AB 相位 */
    int step = rk_enc_table[(enc->state << 2) | curr];
    enc->state = curr;
    enc->pulse += step;

    /* 过滤无效跳变 (step=0 且相位变化了) */
    return (step == 0 && curr != enc->state) ? 0 : step;
}

/**
  * @brief  重置编码器脉冲计数
  */
static inline void rk_encoder_reset(rk_encoder_t *enc)
{
    enc->pulse = 0;
}

#ifdef __cplusplus
}
#endif

#endif /* __RK_KEY_FILTER_H__ */
