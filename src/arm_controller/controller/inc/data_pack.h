#ifndef __DATA_PACK_H__
#define __DATA_PACK_H__

#include <stdint.h>

/**
 * @file data_pack.h
 * @brief 上位机与 MCU 之间 USB CDC 通信的二进制帧定义。
 *
 * 这些结构体是与 MCU 固件共享的 ABI，字段顺序、类型、数量都不能随意改动：
 *  - #pragma pack(1) 保证按字节紧凑排列，编译器不会插入填充字节；
 *  - float 按 IEEE-754 binary32、小端解释，代码不做字节序转换，
 *    要求 MCU 与主机保持一致；
 *  - 每帧固定 6 个电机，上位机启动时会校验关节数，不一致直接拒绝启动；
 *  - 帧内没有长度、校验、序号字段，只靠首字节区分帧类型，负载不做合法性检查。
 *
 * 帧长（pack(1) 下）：
 *  - MCUTarget1Pack：73 字节（1 + 6 × 12）
 *  - MCUTarget2Pack：145 字节（1 + 6 × 24）
 *  - MCUStatePack：73 字节（1 + 6 × 12）
 */

#pragma pack(push, 1)

/**
 * @brief 单电机指令：位置、速度、力矩。
 *
 * 旧版指令帧（0x5A）使用。三个字段都由上位机原样透传，
 * 单位由 MCU 侧约定，上位机不做换算。
 */
typedef struct{
    float position;  ///< 目标位置；单位由 MCU 约定（通常为 rad）
    float velocity;  ///< 目标速度；单位由 MCU 约定（通常为 rad/s）
    float torque;    ///< 前馈力矩；单位由 MCU 约定（通常为 N·m）
}MotorCmdPack;

/**
 * @brief 单电机指令（MIT 阻抗模式）：位置、速度、力矩 + kp/kd/ki。
 *
 * 当前指令帧（0x5B）使用。增益同样原样透传，单位与量纲由 MCU 侧定义；
 * 上位机配置里的增益只是占位值，上电前需要确认 MCU 的增益语义。
 */
typedef struct{
    float position;  ///< 目标位置；单位由 MCU 约定（通常为 rad）
    float velocity;  ///< 目标速度；单位由 MCU 约定（通常为 rad/s）
    float torque;    ///< 前馈力矩；单位由 MCU 约定（通常为 N·m）
    float kp;        ///< 位置刚度增益；单位由 MCU 约定
    float kd;        ///< 速度阻尼增益；单位由 MCU 约定
    float ki;        ///< 积分增益；单位由 MCU 约定
}MotorCmdPackEx;

/**
 * @brief 单电机状态：MCU 回报的实测位置、速度、力矩。
 *
 * 上位机把它映射为 ros2_control 的 position / velocity / effort 三个状态接口。
 * 协议中没有加速度、使能状态、错误码或温度字段；加速度接口由上位机恒置 0。
 */
typedef struct{
    float position;  ///< 实测位置；单位由 MCU 约定（通常为 rad）
    float velocity;  ///< 实测速度；单位由 MCU 约定（通常为 rad/s）
    float torque;    ///< 实测力矩；单位由 MCU 约定（通常为 N·m），映射到 effort
}MotorStatePack;

/**
 * @brief 旧版指令帧，首字节 0x5A，共 73 字节。
 *
 * 只包含位置、速度、力矩，没有增益。当前上位机不使用，仅保留定义；
 * 实际发送的是下面的 MCUTarget2Pack。
 */
typedef struct{
    uint8_t head;             ///< 帧头，固定 0x5A
    MotorCmdPack motor[6];    ///< 6 个电机，下标顺序与关节顺序一致
}MCUTarget1Pack;

/**
 * @brief 当前使用的指令帧，首字节 0x5B，共 145 字节。
 *
 * 每个控制周期由上位机发送一帧（实机配置 200 Hz）。每个电机 24 字节，
 * 字段顺序固定为 position、velocity、torque、kp、kd、ki。
 */
typedef struct{
    uint8_t head;               ///< 帧头，固定 0x5B
    MotorCmdPackEx motor[6];    ///< 6 个电机，下标顺序与关节顺序一致
}MCUTarget2Pack;

/**
 * @brief 状态帧，首字节 0x4A，共 73 字节。
 *
 * 由 MCU 主动上报，上位机异步接收并解析。解析时要求一次收到的字节数
 * 不少于 73，取前 73 字节并校验帧头；超过 100 ms 没收到状态帧即判定通信中断。
 */
typedef struct{
    uint8_t head;               ///< 帧头，固定 0x4A
    MotorStatePack motor[6];    ///< 6 个电机，下标顺序与关节顺序一致
}MCUStatePack;

#pragma pack(pop)
#endif
