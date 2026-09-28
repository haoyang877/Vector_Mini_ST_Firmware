#ifndef __RGB_H__
#define __RGB_H__

#include "main.h"

/*code type of WS2812*/
#define CODE_1 (58)  //count of "1" bit
#define CODE_0 (136) //count of "0" bit

/*total LED number in series*/
#define LED_NUM 1

typedef enum
{
    COLOR_NULL,
    RED,
    GREEN,
    BLUE,
    YELLOW,
    PURPLE,
    CYAN,
    WHITE,
} COLOR_Type;

/*value of R,G,B channel*/
/*larger value leads to brighter light*/
typedef struct
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
} RGB_Color;

/**
 * @brief 把一颗灯的颜色编码到 DMA 缓冲，不立即发送。
 * @param LedId 从零开始的灯索引；调用方限于 0..LED_NUM-1。
 * @param Color 非空颜色，R/G/B 各 0..255；只在本次调用读取。
 * @note 板级单一所有者使用；不得在 DMA 读取缓冲期间覆盖。
 */
void RGB_SetColor(uint8_t LedId, RGB_Color *Color);
/**
 * @brief 把 DMA 缓冲末行清零形成灯带复位间隔。
 * @note 需在提交 DMA 前调用；DMA 使用期间不得修改缓冲。
 */
void Reset_Load(void);
/**
 * @brief 启动 TIM2 通道 3 的 DMA PWM 颜色发送。
 * @note 缓冲由板级模块拥有；调用方必须保证上一 DMA 已结束。HAL 失败未透传。
 */
void RGB_SendArray(void);
/**
 * @brief 把同一颜色编码到指定数量的灯并发送。
 * @param led_num 灯数量，不大于 LED_NUM。
 * @param color 非空只读颜色，R/G/B 各 0..255。
 * @note 修改共享发送缓冲并启动 DMA；调用方保证上一 DMA 已结束。
 */
void write_color(uint16_t led_num, RGB_Color *color);
/**
 * @brief 推进一次呼吸亮度并发送对应颜色。
 * @param color_type 已定义的颜色枚举。
 * @note 单一板级周期任务调用，修改亮度与 DMA 缓冲；不在快速控制中断调用。
 */
void Set_RGB_BreathingColor(COLOR_Type color_type);

#endif