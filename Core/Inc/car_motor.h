#ifndef CAR_MOTOR_H
#define CAR_MOTOR_H
#include <stdint.h>
#include "car_config.h"

/* 电机状态、请求结果与反馈；成功表示请求已接受，执行结果看反馈。 */
typedef enum { MOTOR_INIT, MOTOR_DISABLED, MOTOR_READY, MOTOR_SPEED,
               MOTOR_POSITION, MOTOR_STOPPING, MOTOR_FAULT } MotorState;
/* 上述编号 0~6 是线协议的一部分；STOPPING 需反馈证实零速，FAULT 为锁存故障。 */
typedef struct {
    /* 地址、选项、配置门禁、3A 原始标志、在线、实际使能、加速度档位。 */
    uint8_t addr, option, config_ok, flags, online, enabled, acc;
    MotorState state;
    /* 都是车体方向修正后的电机轴 RPM；目标值与实际反馈可能暂时不同。 */
    int16_t target_rpm, speed_rpm;
    /* Emm 反馈每电机圈 65536 ticks，带符号；不是 POS 指令的细分脉冲数。 */
    int64_t position_ticks;
    /* HAL 毫秒时钟的实际采样时刻；读反馈要同时检查 online/config_ok 与年龄。 */
    uint32_t speed_time, position_time, status_time;
} MotorFeedback;
typedef enum { MOTOR_OK=0, MOTOR_BAD_ARGUMENT=1, MOTOR_NOT_READY=2,
               MOTOR_SAFETY_LOCK=3, MOTOR_BUSY=4 } MotorResult;
/* 参数直接填整数，例如 30、-30、10；电机方向由配置自动转换。 */
/* 初始化电机；now 填 HAL_GetTick() 返回的当前毫秒数。 */
void CarMotor_Init(uint32_t now);
/* 在主循环处理电机任务；now 填 HAL_GetTick() 返回的当前毫秒数。 */
void CarMotor_Poll(uint32_t now);
/* addr 填电机地址，默认左轮1、右轮2；单轮模式只用左轮地址。
 * enable 填1使能、0失能；返回成功后还要等电机反馈确认。 */
MotorResult CarMotor_Enable(uint8_t addr, uint8_t enable);
/* addr 填电机地址，默认左1、右2；rpm 填整数转速，单位 RPM，默认允许 -60~60。
 * 转速正数前进、负数后退、0为零速；acc 填加速度档位0~255，常用10；先使能再调速。 */
MotorResult CarMotor_Velocity(uint8_t addr, int16_t rpm, uint8_t acc);
/* left、right 填左右电机的整数转速，单位 RPM，默认允许 -60~60。
 * 正数前进、负数后退、0为零速；acc 填加速度档位0~255，例如填30、30、10让两轮前进。 */
MotorResult CarMotor_Wheels(int16_t left, int16_t right, uint8_t acc);
/* addr 填电机地址；pulses 填带正负号的整数细分脉冲数。
 * rpm 填正整数转速，单位 RPM，默认1~60；acc 填加速度档位0~255。
 * mode 填0相对上一目标、1绝对坐标、2相对当前位置；位置动作只发送一次。 */
MotorResult CarMotor_Position(uint8_t addr, int32_t pulses, uint16_t rpm, uint8_t acc, uint8_t mode);
/* left、right 填左右轮的整数细分脉冲数，rpm、acc、mode 的填法同上。
 * 相对模式正数向前、负数向后；绝对模式填目标坐标。 */
MotorResult CarMotor_Positions(int32_t left, int32_t right, uint16_t rpm, uint8_t acc, uint8_t mode);
/* 清速度/位置目标，抢占正常事务等待；不截断已开始的 TX，普通 STOP 保留使能请求。 */
void CarMotor_StopAll(void);
/* 已请求使能的零速/停止状态也算活动，仍须按期发心跳。 */
uint8_t CarMotor_Active(void);
/* now 填 HAL_GetTick()（当前未使用）；返回1表示接受恢复或已无故障。
 * 等故障清零后重新使能，再发送运动指令。 */
uint8_t CarMotor_ClearFault(uint32_t now);
/* index 填0读取左轮、1读取右轮，不是电机地址；无对应电机时返回空指针。 */
const MotorFeedback *CarMotor_Get(uint8_t index);

#ifdef CAR_TEST
/* 回归向量使用内部协议接口，不扩展生产控制入口。 */
typedef struct {
    uint8_t data[48];
    uint8_t used;
    uint32_t last_byte;
} X42S_Parser;
uint8_t X42S_Enable(uint8_t *frame, uint8_t addr, uint8_t enable);
uint8_t X42S_EmmVelocity(uint8_t *frame, uint8_t addr, int16_t rpm, uint8_t acc);
uint8_t X42S_EmmPosition(uint8_t *frame, uint8_t addr, uint8_t dir,
                       uint32_t pulses, uint16_t rpm, uint8_t acc, uint8_t mode);
uint8_t X42S_Stop(uint8_t *frame, uint8_t addr);
uint8_t X42S_Query(uint8_t *frame, uint8_t addr, uint8_t code);
uint8_t X42S_ClearProtection(uint8_t *frame, uint8_t addr);
void X42S_ParserReset(X42S_Parser *p);
/* 完整已知回包返回帧长，数据在 p->data；否则返回 0。 */
uint8_t X42S_ParserFeed(X42S_Parser *p, uint8_t byte, uint32_t now);
int64_t X42S_SignedPosition(const uint8_t *frame);
#endif
#endif
