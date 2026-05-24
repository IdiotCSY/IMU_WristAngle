#include "wit_protocol.h"
#include <string.h>

/* 标准重力加速度，用于把 g 单位加速度换算成 m/s^2 */
#define WIT_G_CONST        9.80665f

/* ===================== 内部辅助函数 ===================== */

/*
 * WIT_ToInt16
 *
 * 功能：
 *   将维特帧中的低字节 low、高字节 high 合成为 int16_t。
 *
 * 维特协议采用小端格式：
 *   low  在前
 *   high 在后
 *
 * 例如：
 *   f[2] = AxL
 *   f[3] = AxH
 */
static int16_t WIT_ToInt16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

/*
 * WIT_CheckSum
 *
 * 功能：
 *   计算 11 字节 WIT 数据帧的校验和。
 *
 * 校验规则：
 *   前 10 个字节相加，取低 8 位，应等于第 11 字节 frame[10]。
 */
static uint8_t WIT_CheckSum(const uint8_t *frame)
{
    uint16_t sum = 0;

    for (uint8_t i = 0; i < 10; i++)
    {
        sum += frame[i];
    }

    return (uint8_t)(sum & 0xFF);
}

/*
 * WIT_ParseFrame
 *
 * 功能：
 *   解析一整帧 11 字节 WIT 数据。
 *
 * 注意：
 *   这个函数只处理已经凑齐的一帧。
 *   字节流拼帧由 WIT_ParseByte() 完成。
 */
static void WIT_ParseFrame(WIT_Parser_t *parser, const uint8_t *f)
{
    if (f[0] != WIT_FRAME_HEAD)
    {
        parser->frame_count_bad++;
        return;
    }

    if (WIT_CheckSum(f) != f[10])
    {
        parser->frame_count_bad++;
        return;
    }

    /*
     * 维特数据区通常是 4 个 int16_t：
     *   d0: X 或 q0
     *   d1: Y 或 q1
     *   d2: Z 或 q2
     *   d3: 温度 / 版本 / q3 等，取决于帧类型
     */
    int16_t d0 = WIT_ToInt16(f[2], f[3]);
    int16_t d1 = WIT_ToInt16(f[4], f[5]);
    int16_t d2 = WIT_ToInt16(f[6], f[7]);
    int16_t d3 = WIT_ToInt16(f[8], f[9]);

    switch (f[1])
    {
        case WIT_TYPE_ACC:
        {
            /*
             * 加速度换算：
             *   原始值 / 32768 * 16g
             *
             * 这里进一步乘以 9.80665，换算为 m/s^2。
             */
            parser->acc_mps2[0] = (float)d0 / 32768.0f * 16.0f * WIT_G_CONST;
            parser->acc_mps2[1] = (float)d1 / 32768.0f * 16.0f * WIT_G_CONST;
            parser->acc_mps2[2] = (float)d2 / 32768.0f * 16.0f * WIT_G_CONST;

            parser->frame_count_acc++;
            parser->update_flags |= WIT_FLAG_ACC;
            break;
        }

        case WIT_TYPE_GYRO:
        {
            /*
             * 角速度换算：
             *   原始值 / 32768 * 2000 deg/s
             *
             * 输出单位为 deg/s。
             * 后续 Mahony / GI 使用时，需要再转成 rad/s。
             */
            parser->gyro_deg_s[0] = (float)d0 / 32768.0f * 2000.0f;
            parser->gyro_deg_s[1] = (float)d1 / 32768.0f * 2000.0f;
            parser->gyro_deg_s[2] = (float)d2 / 32768.0f * 2000.0f;

            parser->frame_count_gyro++;
            parser->update_flags |= WIT_FLAG_GYRO;
            break;
        }

        case WIT_TYPE_ANGLE:
        {
            /*
             * 欧拉角换算：
             *   原始值 / 32768 * 180 deg
             *
             * 注意：
             *   当前 MCU 主算法暂时不依赖原厂欧拉角。
             *   解析它主要用于调试或和原厂算法对比。
             */
            parser->angle_deg[0] = (float)d0 / 32768.0f * 180.0f;
            parser->angle_deg[1] = (float)d1 / 32768.0f * 180.0f;
            parser->angle_deg[2] = (float)d2 / 32768.0f * 180.0f;

            parser->frame_count_angle++;
            parser->update_flags |= WIT_FLAG_ANGLE;
            break;
        }

        case WIT_TYPE_QUAT:
        {
            /*
             * 四元数换算：
             *   原始值 / 32768
             *
             * 格式：
             *   quat[0] = q0 = w
             *   quat[1] = q1 = x
             *   quat[2] = q2 = y
             *   quat[3] = q3 = z
             */
            parser->quat[0] = (float)d0 / 32768.0f;
            parser->quat[1] = (float)d1 / 32768.0f;
            parser->quat[2] = (float)d2 / 32768.0f;
            parser->quat[3] = (float)d3 / 32768.0f;

            parser->frame_count_quat++;
            parser->update_flags |= WIT_FLAG_QUAT;
            break;
        }

        default:
        {
            /*
             * 未支持的数据帧类型。
             * 例如磁场、端口状态、气压等，如果后续要用，可以继续扩展。
             */
            parser->frame_count_bad++;
            break;
        }
    }
}

/* ===================== 对外函数实现 ===================== */

void WIT_Init(WIT_Parser_t *parser)
{
    if (parser == 0)
    {
        return;
    }

    memset(parser, 0, sizeof(WIT_Parser_t));

    /* 默认四元数初始化为单位四元数 */
    parser->quat[0] = 1.0f;
    parser->quat[1] = 0.0f;
    parser->quat[2] = 0.0f;
    parser->quat[3] = 0.0f;
}

void WIT_ClearFlags(WIT_Parser_t *parser)
{
    if (parser == 0)
    {
        return;
    }

    parser->update_flags = 0;
}

void WIT_ParseByte(WIT_Parser_t *parser, uint8_t byte)
{
    if (parser == 0)
    {
        return;
    }

    /*
     * index = 0 时，说明当前还没找到帧头。
     * 只有收到 0x55，才开始收一帧。
     */
    if (parser->index == 0)
    {
        if (byte == WIT_FRAME_HEAD)
        {
            parser->frame[0] = byte;
            parser->index = 1;
        }

        return;
    }

    /*
     * 已经找到帧头后，继续收后续字节。
     */
    parser->frame[parser->index] = byte;
    parser->index++;

    /*
     * 凑够 11 字节后，尝试解析。
     * 无论解析成功或失败，都重新等待下一帧。
     */
    if (parser->index >= WIT_FRAME_LEN)
    {
        WIT_ParseFrame(parser, parser->frame);
        parser->index = 0;
    }
}

void WIT_ParseBuffer(WIT_Parser_t *parser, const uint8_t *data, uint16_t len)
{
    if (parser == 0 || data == 0)
    {
        return;
    }

    /*
     * DMA 一次可能收到任意长度数据：
     *   可能刚好是一帧；
     *   可能是多帧；
     *   也可能是半帧。
     *
     * 所以这里逐字节送入状态机，未完成的半帧会保存在 parser->frame 中。
     */
    for (uint16_t i = 0; i < len; i++)
    {
        WIT_ParseByte(parser, data[i]);
    }
}

uint8_t WIT_HasAcc(const WIT_Parser_t *parser)
{
    return (parser != 0 && (parser->update_flags & WIT_FLAG_ACC)) ? 1U : 0U;
}

uint8_t WIT_HasGyro(const WIT_Parser_t *parser)
{
    return (parser != 0 && (parser->update_flags & WIT_FLAG_GYRO)) ? 1U : 0U;
}

uint8_t WIT_HasAngle(const WIT_Parser_t *parser)
{
    return (parser != 0 && (parser->update_flags & WIT_FLAG_ANGLE)) ? 1U : 0U;
}

uint8_t WIT_HasQuat(const WIT_Parser_t *parser)
{
    return (parser != 0 && (parser->update_flags & WIT_FLAG_QUAT)) ? 1U : 0U;
}
