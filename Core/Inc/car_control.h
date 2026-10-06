#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H
#include "main.h"

/* 初始化整车；motor 填 &huart1 连接电机，host 填 &huart2 连接上位机，串口须先初始化。 */
void CarControl_Init(UART_HandleTypeDef *motor, UART_HandleTypeDef *host);
/* 在 while(1) 每轮调用；不能通过 HAL_Delay 放慢此入口来调电机速度。 */
void CarControl_Poll(void);

/* 跨模块保护接口；Clear 仅在电机异步恢复已确认安全后使用。 */
enum {
    FAULT_ESTOP=1U, FAULT_HOST_TIMEOUT=2U, FAULT_MOTOR_COMM=4U,
    FAULT_DRIVER=8U, FAULT_CONFIG=16U, FAULT_UART=32U, FAULT_COMMAND=64U
};
/* 更新心跳时间；now 填 HAL_GetTick() 返回的当前毫秒数。 */
void CarControl_Heartbeat(uint32_t now);
/* fault 填上面的 FAULT_* 故障值，多项故障用 | 合并；记录故障后停机。 */
void CarControl_Trip(uint16_t fault);
/* 返回当前故障，0表示无故障；用 返回值 & FAULT_* 判断某项故障。 */
uint16_t CarControl_Faults(void);
/* motors_safe 填0表示未通过安全检查，填1表示已通过；now 填 HAL_GetTick()。
 * 仅供电机恢复流程调用，返回1表示已清故障。 */
uint8_t CarControl_Clear(uint8_t motors_safe, uint32_t now);
/* 无参数；返回 1=已开启的低有效保护输入全部释放，0=仍有触发；不检查电机反馈。 */
uint8_t CarControl_InputsReleased(void);
#endif
