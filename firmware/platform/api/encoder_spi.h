#ifndef ENCODER_SPI_H
#define ENCODER_SPI_H

#include <stdbool.h>
#include <stdint.h>

/* 编码器 SPI 传输契约：对传感器做一次半双工 16 位寄存器读取。
 * 请求帧与读帧内容由传感器驱动给出；本契约只负责片选、收发、方向切换与超时，
 * 不暴露 SPI 句柄、寄存器或引脚。20 kHz 快速环可先 begin 后 complete，
 * 两次调用之间禁止访问同一总线。 */

/**
 * @brief 初始化编码器 SPI 外设与引脚。
 * @note 上电初始化调用一次；初始化失败按既有致命错误路径处理。
 */
void encoder_spi_init(void);

/**
 * @brief 发起一次寄存器读取的请求帧，不等待传输完成。
 * @param request_frame 传感器驱动给出的 16 位请求帧。
 * @return 总线就绪且已发出请求返回 true；外设未使能或总线忙返回 false，
 *         此时未拉低片选，调用方应改用同步路径。
 * @note 20 kHz 快速环调用；返回 true 后必须由同一调用方完成本帧。
 */
bool encoder_spi_read_begin(uint16_t request_frame);

/* 本拍栈上的事务阶段；0/1兼容原begin返回值，无跨拍可变状态。 */
#define ENCODER_SPI_READ_SYNC 0U
#define ENCODER_SPI_READ_COMMAND_PENDING 1U
#define ENCODER_SPI_READ_RESPONSE_PENDING 2U
#define ENCODER_SPI_READ_FAILED 3U

/**
 * @brief 有界收回请求帧并发起读帧，不等待数据响应。
 * @param begin_ok 本拍encoder_spi_read_begin返回值，仅推进一次。
 * @param read_frame 读相位发送的16位帧。
 * @return 本拍事务阶段，必须原样交给encoder_spi_read_complete收尾。
 * @note 仅快环独占总线时调用；失败不补发，仍由complete释放CS/MOSI。
 */
uint8_t encoder_spi_read_begin_readback(bool begin_ok, uint16_t read_frame);

/**
 * @brief 完成一次寄存器读取：读回请求响应，切换读相位并取回数据帧。
 * @param sample_stage 本拍begin的0/1返回值或begin_readback的阶段；同步阶段自行补发请求。
 * @param request_frame 传感器驱动给出的 16 位请求帧。
 * @param read_frame 读相位发送的 16 位帧（无数据时通常为 0）。
 * @param word 输出传感器返回的 16 位数据帧；仅返回 true 时有效。
 * @return 完整读回返回 true；任一阶段超时返回 false。
 * @note 20 kHz 快速环调用；无论成功失败都释放片选与 MOSI 方向。
 */
bool encoder_spi_read_complete(uint8_t sample_stage,
                               uint16_t request_frame,
                               uint16_t read_frame,
                               uint16_t *word);

/**
 * @brief 同步读取一个传感器寄存器（只读，诊断用）。
 * @param request_frame 传感器驱动给出的 16 位请求帧。
 * @param word 输出 16 位数据帧；仅返回 true 时有效。
 * @return 完整读回返回 true；总线未就绪或任一阶段超时返回 false。
 * @note 阻塞式；仅供初始化/诊断读取，不得在 20 kHz 快速环调用。
 */
bool encoder_spi_read_register(uint16_t request_frame, uint16_t *word);

#endif
