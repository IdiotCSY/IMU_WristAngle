#ifndef __WIT_PROTOCOL_H
#define __WIT_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== WIT 数据帧基础定义 ===================== */

/*
 * 维特标准数据帧长度固定为 11 字节：
 *
 * Byte0  : 帧头，固定 0x55
 * Byte1  : 数据类型，例如 0x51 加速度，0x52 角速度
 * Byte2~9: 数据区，共 8 字节，通常为 4 个 int16_t
 * Byte10 : 校验和
 */
#define WIT_FRAME_LEN      11
#define WIT_FRAME_HEAD     0x55

/* 数据类型 */
#define WIT_TYPE_ACC       0x51    /* 加速度 */
#define WIT_TYPE_GYRO      0x52    /* 角速度 */
#define WIT_TYPE_ANGLE     0x53    /* 欧拉角 */
#define WIT_TYPE_QUAT      0x59    /* 四元数 */

/* update_flags 标志位：用于判断最近是否收到某类数据 */
#define WIT_FLAG_ACC       (1U << 0)
#define WIT_FLAG_GYRO      (1U << 1)
#define WIT_FLAG_ANGLE     (1U << 2)
#define WIT_FLAG_QUAT      (1U << 3)

/* ===================== WIT 解析器结构体 ===================== */

typedef struct
{
    /*
     * frame:
     *   当前正在拼接的 11 字节数据帧。
     *
     * index:
     *   当前已经接收到的数据帧字节位置。
     *   index = 0 表示正在等待帧头 0x55。
     */
    uint8_t frame[WIT_FRAME_LEN];
    uint8_t index;

    /*
     * 解析后的物理量。
     *
     * acc_mps2:
     *   加速度，单位 m/s^2。
     *
     * gyro_deg_s:
     *   角速度，单位 deg/s。
     *
     * angle_deg:
     *   欧拉角，单位 deg。
     *
     * quat:
     *   四元数，格式 [q0 q1 q2 q3] = [w x y z]。
     */
    float acc_mps2[3];
    float gyro_deg_s[3];
    float angle_deg[3];
    float quat[4];

    /*
     * 各类数据帧计数。
     * 后续可以用来统计是否丢包、是否达到设定回传频率。
     */
    uint32_t frame_count_acc;
    uint32_t frame_count_gyro;
    uint32_t frame_count_angle;
    uint32_t frame_count_quat;
    uint32_t frame_count_bad;

    /*
     * update_flags:
     *   最近一次解析过程中，哪些数据被更新过。
     *   例如：
     *      update_flags & WIT_FLAG_GYRO 非零，说明收到过新的角速度帧。
     *
     * 使用后可以调用 WIT_ClearFlags() 清空。
     */
    uint32_t update_flags;

} WIT_Parser_t;

/* ===================== 对外函数声明 ===================== */

/* 初始化解析器 */
void WIT_Init(WIT_Parser_t *parser);

/* 清空 update_flags，不清空已经解析出的 acc / gyro 等数据 */
void WIT_ClearFlags(WIT_Parser_t *parser);

/* 输入 1 个字节，解析器内部自动拼帧 */
void WIT_ParseByte(WIT_Parser_t *parser, uint8_t byte);

/* 输入一段字节流，适合 DMA 收到一批数据后调用 */
void WIT_ParseBuffer(WIT_Parser_t *parser, const uint8_t *data, uint16_t len);

/* 判断最近是否收到对应数据 */
uint8_t WIT_HasAcc(const WIT_Parser_t *parser);
uint8_t WIT_HasGyro(const WIT_Parser_t *parser);
uint8_t WIT_HasAngle(const WIT_Parser_t *parser);
uint8_t WIT_HasQuat(const WIT_Parser_t *parser);

#ifdef __cplusplus
}
#endif

#endif
