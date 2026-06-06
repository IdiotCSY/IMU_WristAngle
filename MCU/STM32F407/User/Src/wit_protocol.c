#include "wit_protocol.h"
#include <string.h>

/*
 * 维特加速度帧默认量程是 ±16g。
 * 解析时先得到 g，再乘以标准重力加速度，得到 m/s^2。
 */
#define WIT_G_CONST        9.80665f

/* ===================== 内部辅助函数 ===================== */

static int16_t WIT_ToInt16(uint8_t low, uint8_t high)
{
    /*
     * WIT 协议采用小端格式：
     *
     *   low  在前
     *   high 在后
     */
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

static uint8_t WIT_CheckSum(const uint8_t *frame)
{
    /*
     * 校验规则：
     *   前 10 个字节相加，取低 8 位，
     *   应等于第 11 个字节 frame[10]。
     */
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
 * 返回：
 *   WIT_TYPE_NONE:
 *      帧无效，或者帧类型不支持。
 *
 *   WIT_TYPE_ACC / WIT_TYPE_GYRO / ...
 *      成功解析出的帧类型。
 *
 * 重要：
 *   如果返回 GYRO，说明 parser->gyro_deg_s 已经是最新值。
 *   如果返回 ACC，说明 parser->acc_mps2 已经是最新值。
 */
static uint8_t WIT_ParseFrame(WIT_Parser_t *parser, const uint8_t *f)
{
    if (f[0] != WIT_FRAME_HEAD)
    {
        parser->frame_count_bad++;
        return WIT_TYPE_NONE;
    }

    if (WIT_CheckSum(f) != f[10])
    {
        parser->frame_count_bad++;
        return WIT_TYPE_NONE;
    }

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
             *
             *   raw / 32768 * 16g * 9.80665
             *
             * 输出单位：
             *   m/s^2
             */
            parser->acc_mps2[0] = (float)d0 / 32768.0f * 16.0f * WIT_G_CONST;
            parser->acc_mps2[1] = (float)d1 / 32768.0f * 16.0f * WIT_G_CONST;
            parser->acc_mps2[2] = (float)d2 / 32768.0f * 16.0f * WIT_G_CONST;

            parser->frame_count_acc++;
            parser->update_flags |= WIT_FLAG_ACC;

            return WIT_TYPE_ACC;
        }

        case WIT_TYPE_GYRO:
        {
            /*
             * 角速度换算：
             *
             *   raw / 32768 * 2000 deg/s
             *
             * 输出单位：
             *   deg/s
             */
            parser->gyro_deg_s[0] = (float)d0 / 32768.0f * 2000.0f;
            parser->gyro_deg_s[1] = (float)d1 / 32768.0f * 2000.0f;
            parser->gyro_deg_s[2] = (float)d2 / 32768.0f * 2000.0f;

            parser->frame_count_gyro++;
            parser->update_flags |= WIT_FLAG_GYRO;

            return WIT_TYPE_GYRO;
        }

        case WIT_TYPE_ANGLE:
        {
            /*
             * 欧拉角换算：
             *
             *   raw / 32768 * 180 deg
             *
             * 当前 MCU 主算法不用原厂欧拉角。
             * 解析它主要用于调试或对比。
             */
            parser->angle_deg[0] = (float)d0 / 32768.0f * 180.0f;
            parser->angle_deg[1] = (float)d1 / 32768.0f * 180.0f;
            parser->angle_deg[2] = (float)d2 / 32768.0f * 180.0f;

            parser->frame_count_angle++;
            parser->update_flags |= WIT_FLAG_ANGLE;

            return WIT_TYPE_ANGLE;
        }

        case WIT_TYPE_QUAT:
        {
            /*
             * 原厂四元数换算：
             *
             *   raw / 32768
             *
             * 格式：
             *   quat[0] = w
             *   quat[1] = x
             *   quat[2] = y
             *   quat[3] = z
             */
            parser->quat[0] = (float)d0 / 32768.0f;
            parser->quat[1] = (float)d1 / 32768.0f;
            parser->quat[2] = (float)d2 / 32768.0f;
            parser->quat[3] = (float)d3 / 32768.0f;

            parser->frame_count_quat++;
            parser->update_flags |= WIT_FLAG_QUAT;

            return WIT_TYPE_QUAT;
        }

        default:
        {
            /*
             * 其他帧类型暂时不支持。
             */
            parser->frame_count_bad++;
            return WIT_TYPE_NONE;
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

    /*
     * 默认四元数初始化为单位四元数。
     */
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
    /*
     * 兼容旧接口。
     * 不关心返回值。
     */
    (void)WIT_ParseByteEx(parser, byte);
}

uint8_t WIT_ParseByteEx(WIT_Parser_t *parser, uint8_t byte)
{
    if (parser == 0)
    {
        return WIT_TYPE_NONE;
    }

    /*
     * index = 0：
     *   还没有找到帧头。
     *
     * 只有收到 0x55，才开始收一帧。
     */
    if (parser->index == 0)
    {
        if (byte == WIT_FRAME_HEAD)
        {
            parser->frame[0] = byte;
            parser->index = 1;
        }

        return WIT_TYPE_NONE;
    }

    /*
     * 已经找到帧头，继续保存后续字节。
     */
    parser->frame[parser->index] = byte;
    parser->index++;

    /*
     * 凑够 11 字节后，尝试解析。
     */
    if (parser->index >= WIT_FRAME_LEN)
    {
        uint8_t frame_type = WIT_ParseFrame(parser, parser->frame);

        /*
         * 无论解析成功或失败，都重新等待下一帧。
         */
        parser->index = 0;

        return frame_type;
    }

    return WIT_TYPE_NONE;
}

void WIT_ParseBuffer(WIT_Parser_t *parser, const uint8_t *data, uint16_t len)
{
    if (parser == 0 || data == 0)
    {
        return;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        WIT_ParseByte(parser, data[i]);
    }
}

void WIT_ParseBufferWithCallback(WIT_Parser_t *parser,
                                 const uint8_t *data,
                                 uint16_t len,
                                 WIT_FrameCallback_t callback,
                                 void *user_data)
{
    if (parser == 0 || data == 0)
    {
        return;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        /*
         * 每输入一个字节，都检查这一次是否刚好解析出一帧。
         */
        uint8_t frame_type = WIT_ParseByteEx(parser, data[i]);

        /*
         * 如果解析出有效帧，并且用户提供了回调函数，
         * 就通知外层模块。
         *
         * 此时 parser 内部对应的数据已经更新完成。
         */
        if (frame_type != WIT_TYPE_NONE && callback != 0)
        {
            callback(parser, frame_type, user_data);
        }
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
