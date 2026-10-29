#ifndef __DATA_PACK_H__
#define __DATA_PACK_H__

#include <stdint.h>

/**
 * @file data_pack.h
 * @brief 上位机与 MCU 之间 USB CDC 通信的二进制帧定义。
 *
 * 这些结构体是**下位机固件**（2026_R2_NEW_ARM_LOWER/MyTask/Run.h）的字节级镜像，
 * 字段顺序、类型、数量都不能单方面改动：
 *  - 固件用 `#pragma pack(1)`，本文件同样按 1 字节对齐；
 *  - 固件帧头是 `int pack_type`（4 字节，小端），取值固定 0x01。
 *    注意不是单字节，也不是 0x5A/0x5B/0x4A——旧版上位机的那套帧固件里不存在；
 *  - float 按 IEEE-754 binary32、小端解释，代码不做字节序转换；
 *  - 每帧固定 6 个电机槽，但固件实际只驱动前 5 个（joint5 之后的槽位固件既不读也不写）；
 *  - 帧内没有长度、校验、序号字段，只靠帧头区分，负载不做合法性检查。
 *
 * 帧长（pack(1) 下）：
 *  - MCUTargetPack：78 字节（4 + 6 × 12 + 1 + 1）
 *  - MCUStatePack：77 字节（4 + 6 × 12 + 1）
 *
 * 注意 USB FS 的 bulk 传输上限是 64 字节，77 字节的上行帧在主机侧必然被拆成
 * 64 + 13 两次回调，接收侧必须自己做字节流累加（见 ArmRealInterfaces::on_pack_praser）。
 */

#pragma pack(push, 1)

/**
 * @brief 单电机数据：位置、速度、力矩（角度/角速度/力矩）。
 *
 * 位置单位 rad，固件侧还会再套一层「零位偏置 + 方向」，上位机看到的角度
 * 已经过 joint_calibration.yaml 的 dir/zero 换算，见 ArmRealInterfaces。
 */
typedef struct {
    float rad;      ///< 关节角度 rad
    float omega;    ///< 关节角速度 rad/s
    float torque;   ///< 输出力矩 N·m
} MotorPack;

/**
 * @brief 下行指令帧：PC -> MCU，78 字节（对应固件 ArmTarget_t）。
 *
 * 每个控制周期发送一帧（实机配置 100 Hz）。
 */
typedef struct {
    int32_t   pack_type;   ///< 帧头，固定 0x01（固件按 int 读取，必须 4 字节）
    MotorPack joint[6];    ///< 6 个电机槽；固件只使用前 5 个
    uint8_t   air_pump;    ///< 气泵开关：1 = 开，0 = 关（当前上位机恒发 0）
    uint8_t   worked;      ///< 一段任务结束标志（当前上位机恒发 0）
} MCUTargetPack;

/**
 * @brief 上行状态帧：MCU -> PC，77 字节（对应固件 ArmState_t）。
 *
 * 由 MCU 主动上报，周期 20 ms（50 Hz）。上位机异步接收并拼帧解析，
 * 超过 100 ms 没收到完整状态帧即判定通信中断。
 */
typedef struct {
    int32_t   pack_type;   ///< 帧头，固定 0x01
    MotorPack joint[6];    ///< 6 个电机槽；固件只写前 5 个，第 6 槽恒 0
    uint8_t   message;     ///< 抓取等状态消息（固件目前只在遥控器路径置位）
} MCUStatePack;

#pragma pack(pop)

static_assert(sizeof(MotorPack) == 12, "MotorPack 必须与固件 Motor_t 同为 12 字节");
static_assert(sizeof(MCUTargetPack) == 78, "下行帧必须与固件 ArmTarget_t 同为 78 字节");
static_assert(sizeof(MCUStatePack) == 77, "上行帧必须与固件 ArmState_t 同为 77 字节");
static_assert(sizeof(MCUTargetPack::joint) / sizeof(MotorPack) == 6, "下行帧固定 6 个电机槽");
static_assert(sizeof(MCUStatePack::joint) / sizeof(MotorPack) == 6, "上行帧固定 6 个电机槽");

#endif
