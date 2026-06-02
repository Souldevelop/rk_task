/**
  * @file    rk_key_filter.c
  * @brief   rktask 按键滤波 & 旋转编码器解码 - 高级扫描接口
  * @author  Reverseking
  * @note    提供与 rktask 事件标志集成的高层扫描函数
  */
#include "rk_key_filter.h"
#include "rk_task.h"

/* ======================== 按键批量扫描 ======================== */

/**
  * @brief  扫描按键并发布事件
  * @param  btn      按键实例指针
  * @param  active   当前 GPIO 电平 (0=按下)
  * @param  evt_id   目标事件组 ID
  * @param  key_bit  按键事件位索引 (传给 RK_BTN_DOWN/UP/LONG)
  *
  * 内部记录上次状态, 在按下/释放时自动发布事件.
  * 典型用法: 每 5~10ms 调用一次.
  */
void rk_btn_scan_post(rk_btn_t *btn, uint32_t active,
                      rk_id_t evt_id, uint8_t key_idx)
{
    static uint32_t last_btn[4];   /* 记录每个按键上次状态, 支持最多4个 */
    uint32_t *plast = &last_btn[key_idx & 3];

    uint32_t curr = rk_btn_scan(btn, active);

    if (curr && !*plast)
    {
        /* 释放→按下 */
        rk_task_evt_post(evt_id, RK_BTN_DOWN(key_idx));
    }
    else if (!curr && *plast)
    {
        /* 按下→释放 */
        rk_task_evt_post(evt_id, RK_BTN_UP(key_idx));
    }
    *plast = curr;
}

/* ======================== 编码器批量扫描 ======================== */

/**
  * @brief  扫描编码器并发布事件
  * @param  enc     编码器实例指针
  * @param  a       A 相电平
  * @param  b       B 相电平
  * @param  evt_id  目标事件组 ID
  * @param  enc_idx 编码器索引 (传给 RK_ENC_CW/CCW)
  *
  * 每次检测到有效的旋转步进时发布事件.
  */
void rk_encoder_scan_post(rk_encoder_t *enc, uint8_t a, uint8_t b,
                          rk_id_t evt_id, uint8_t enc_idx)
{
    int step = rk_encoder_scan(enc, a, b);

    if (step > 0)
    {
        rk_task_evt_post(evt_id, RK_ENC_CW(enc_idx));
    }
    else if (step < 0)
    {
        rk_task_evt_post(evt_id, RK_ENC_CCW(enc_idx));
    }
}
