#ifndef __WIT_PROTOCOL_H
#define __WIT_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== WIT 数据帧基础定义 ===================== */

#define WIT_FRAME_LEN      11
#define WIT_FRAME_HEAD     0x55

#define WIT_TYPE_NONE      0x00
#define WIT_TYPE_ACC       0x51
#define WIT_TYPE_GYRO      0x52
#define WIT_TYPE_ANGLE     0x53
#define WIT_TYPE_QUAT      0x59

#define WIT_FLAG_ACC       (1U << 0)
#define WIT_FLAG_GYRO      (1U << 1)
#define WIT_FLAG_ANGLE     (1U << 2)
#define WIT_FLAG_QUAT      (1U << 3)

/*
 * 前向声明。
 *
 * 下面的回调函数类型里要用到 WIT_Parser_t，
 * 所以这里先声明一下。
 */
typedef struct WIT_Parser WIT_Parser_t;

/*
 * WIT 帧解析回调函数类型。
 *
 * 功能：
 *   当 WIT_ParseBufferWithCallback() 解析到一帧有效数据时，
 *   就会调用这个回调函数。
 *
 * 参数：
 *   parser:
 *      当前解析器。
 *      回调被调用时，parser 内部的 acc / gyro 等数据已经更新完成。
 *
 *   frame_type:
 *      当前解析到的帧类型。
 *      例如：
 *        WIT_TYPE_ACC
 *        WIT_TYPE_GYRO
 *
 *   user_data:
 *      用户自定义指针。
 *      后面 imu_uart.c 可以用它传递 hand/arm 标记，或者暂时不用。
 */
typedef void (*WIT_FrameCallback_t)(WIT_Parser_t *parser,
                                    uint8_t frame_type,
                                    void *user_data);

/* ===================== WIT 解析器结构体 ===================== */

struct WIT_Parser
{
    /*
     * 当前正在拼接的一帧数据。
     *
     * WIT 协议帧固定为 11 字节。
     */
    uint8_t frame[WIT_FRAME_LEN];

    /*
     * 当前已经接收到第几个字节。
     *
     * index = 0：
     *   正在等待帧头 0x55。
     *
     * index > 0：
     *   已经找到帧头，正在接收后续字节。
     */
    uint8_t index;

    /*
     * 解析后的物理量。
     */
    float acc_mps2[3];      /* 加速度，单位 m/s^2 */
    float gyro_deg_s[3];    /* 角速度，单位 deg/s */
    float angle_deg[3];     /* 欧拉角，单位 deg */
    float quat[4];          /* 四元数，[w x y z] */

    /*
     * 各类帧计数。
     *
     * 这些计数可以用于判断：
     *   1. 数据是否在持续更新；
     *   2. 是否丢帧；
     *   3. bad frame 是否异常增加。
     */
    uint32_t frame_count_acc;
    uint32_t frame_count_gyro;
    uint32_t frame_count_angle;
    uint32_t frame_count_quat;
    uint32_t frame_count_bad;

    /*
     * 最近一次解析过程中更新了哪些数据。
     */
    uint32_t update_flags;
};

/* ===================== 基础解析函数 ===================== */

void WIT_Init(WIT_Parser_t *parser);
void WIT_ClearFlags(WIT_Parser_t *parser);

/*
 * 旧版接口：
 *   输入 1 个字节，内部自动拼帧。
 *   不关心这次是否刚好解析出有效帧。
 */
void WIT_ParseByte(WIT_Parser_t *parser, uint8_t byte);

/*
 * 新版字节接口：
 *   输入 1 个字节。
 *
 * 返回：
 *   WIT_TYPE_NONE:
 *      本次没有解析出有效帧，或者帧无效。
 *
 *   WIT_TYPE_ACC / WIT_TYPE_GYRO / ...
 *      本次刚好解析出一帧有效数据。
 */
uint8_t WIT_ParseByteEx(WIT_Parser_t *parser, uint8_t byte);

/*
 * 旧版 buffer 接口：
 *   一次输入一段字节流。
 *   只更新 parser 内部最新 acc / gyro。
 */
void WIT_ParseBuffer(WIT_Parser_t *parser, const uint8_t *data, uint16_t len);

/*
 * 新版 buffer + 回调接口：
 *
 * 功能：
 *   一次输入一段字节流。
 *   每解析出一帧有效数据，就调用 callback。
 *
 * 用途：
 *   后面我们要在 imu_uart.c 里：
 *
 *      解析到 ACC  帧：更新 latestAcc
 *      解析到 GYRO 帧：生成一个 IMU sample，放入队列
 *
 * 这样就能做到：
 *
 *      一个 GYRO 帧 = 一次 Mahony 更新
 */
void WIT_ParseBufferWithCallback(WIT_Parser_t *parser,
                                 const uint8_t *data,
                                 uint16_t len,
                                 WIT_FrameCallback_t callback,
                                 void *user_data);

/* ===================== 标志位查询函数 ===================== */

uint8_t WIT_HasAcc(const WIT_Parser_t *parser);
uint8_t WIT_HasGyro(const WIT_Parser_t *parser);
uint8_t WIT_HasAngle(const WIT_Parser_t *parser);
uint8_t WIT_HasQuat(const WIT_Parser_t *parser);

#ifdef __cplusplus
}
#endif

#endif
