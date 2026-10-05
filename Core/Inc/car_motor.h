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
void CarMotor_Init(uint32_t now);
/* 主循环推进查询/目标/恢复；每次最多提交一个普通事务，不阻塞等待 ACK。 */
void CarMotor_Poll(uint32_t now);
/* enable=1 请求使能；=0 先停双轮，再失能指定地址。返回 MOTOR_OK 仅为接受。 */
MotorResult CarMotor_Enable(uint8_t addr, uint8_t enable);
/* rpm 带符号且受 CAR_MAX_RPM 限制，acc 为 0~255 的驱动器档位。 */
MotorResult CarMotor_Velocity(uint8_t addr, int16_t rpm, uint8_t acc);
/* 先检查两轮，全部通过才更新目标；硬件实际逐台发送，不是同步广播。 */
MotorResult CarMotor_Wheels(int16_t left, int16_t right, uint8_t acc);
/* pulses 为带符号细分脉冲，rpm=1..上限；mode 0=相对上一输入目标，
 * 1=绝对坐标，2=相对实时位置。只发送一次，ACK 丢失后故障停机，不重发。 */
MotorResult CarMotor_Position(uint8_t addr, int32_t pulses, uint16_t rpm, uint8_t acc, uint8_t mode);
MotorResult CarMotor_Positions(int32_t left, int32_t right, uint16_t rpm, uint8_t acc, uint8_t mode);
/* 清速度/位置目标，抢占正常事务等待；不截断已开始的 TX，普通 STOP 保留使能请求。 */
void CarMotor_StopAll(void);
/* 已请求使能的零速/停止状态也算活动，仍须按期发心跳。 */
uint8_t CarMotor_Active(void);
/* 返回 1 表示恢复已接受/进行中或已无故障；必须等 Faults()==0，随后重新使能。 */
uint8_t CarMotor_ClearFault(uint32_t now);
/* index=0 左轮、1 右轮；单轮模式 index=1 返回空指针，只读内部缓存。 */
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
