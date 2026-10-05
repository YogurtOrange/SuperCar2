#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H
#include "main.h"

/* main.c 只调用这两个整车入口；板级初始化与保护调度在本模块内。 */
void CarControl_Init(UART_HandleTypeDef *motor, UART_HandleTypeDef *host);
/* 在 while(1) 每轮调用；不能通过 HAL_Delay 放慢此入口来调电机速度。 */
void CarControl_Poll(void);

/* 跨模块保护接口；Clear 仅在电机异步恢复已确认安全后使用。 */
enum {
    FAULT_ESTOP=1U, FAULT_HOST_TIMEOUT=2U, FAULT_MOTOR_COMM=4U,
    FAULT_DRIVER=8U, FAULT_CONFIG=16U, FAULT_UART=32U, FAULT_COMMAND=64U
};
/* fault 是可组合的位掩码；例如 6 表示主机超时(2)与电机通信(4)同时发生。 */
/* 记录有效保活时间，不能清故障；协议层决定哪些已接受请求允许调用。 */
void CarControl_Heartbeat(uint32_t now);
/* 按位锁存故障，由随后电机调度触发停止；不是同步硬件断电。 */
void CarControl_Trip(uint16_t fault);
uint16_t CarControl_Faults(void);
/* 仅供 car_motor 恢复流程使用，不要从 main 直接强行传 1 绕过电机核验。 */
uint8_t CarControl_Clear(uint8_t motors_safe, uint32_t now);
/* 检查已开启的低有效输入是否全部释放；不检查电机反馈。 */
uint8_t CarControl_InputsReleased(void);
#endif
