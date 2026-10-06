#ifndef CAR_UART_H
#define CAR_UART_H
#include "main.h"
#include "car_config.h"

/* 两路 UART 的非阻塞收发；调用者不得在中断里解析控制命令。 */
typedef enum { UART_MOTOR=0, UART_HOST=1 } UartPort;
/* motor 填 &huart1，host 填 &huart2；先完成这两个串口的初始化。 */
void CarUart_Init(UART_HandleTypeDef *motor, UART_HandleTypeDef *host);
/* 维护 TX 队列、接收重挂与卡死检测；不解析主机命令。 */
void CarUart_Poll(void);
/* port 填 UART_MOTOR 读取电机串口，或 UART_HOST 读取上位机串口。
 * byte 填接收字节变量的地址，如 &b；返回1表示读到字节，0表示没有数据。 */
uint8_t CarUart_Read(UartPort port, uint8_t *byte);
/* data 填保存完整 Emm 指令的字节数组，不填 "01 F6 ..." 这样的文字。
 * length 填实际发送字节数，最多16且不超过数组长度；返回1表示开始发送。 */
uint8_t CarUart_SendMotor(const uint8_t *data, uint8_t length);
/* 无参数；返回 1=USART1 尚有在途发送，0=没有在途发送；0 不保证帧间隔已到。 */
uint8_t CarUart_MotorTxBusy(void);
/* port 填 UART_MOTOR 或 UART_HOST；读取并清除该串口故障标志，返回1有故障、0无故障。 */
uint8_t CarUart_TakeRxFault(UartPort port);
/* port 填 UART_MOTOR 或 UART_HOST；丢弃该串口尚未处理的接收数据。 */
void CarUart_FlushRx(UartPort port);

/* 主循环分开调用维护与命令处理，保证保护检查先于命令执行。 */
void CarUart_HostInit(void);
/* now 填 HAL_GetTick()；从 USART2 读取上位机指令。
 * 文本例子：WHEELS 30 30 10\r\n；二进制速度数据为左右RPM各2字节（低字节在前）和加速度1字节。
 * 二进制完整格式见 docs/上位机协议_v1.md；同一会话使用一种格式。 */
void CarUart_HostPoll(uint32_t now);
/* now 填 HAL_GetTick()；把已收到的电机反馈发送给上位机。 */
void CarUart_HostReport(uint32_t now);

#ifdef CAR_TEST
/* 仅原生回归开放编解码接口，生产固件的解析状态留在模块内部。 */
typedef struct {
    uint8_t data[CAR_HOST_MAX_PAYLOAD+8];
    uint8_t used;
    uint32_t last_byte;
} HostCodec;
uint16_t HostCodec_Crc16(const uint8_t *data, uint8_t length);
uint8_t HostCodec_Encode(uint8_t *frame, uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t length);
/* 1=有效完整帧，0=还未完整，-1=丢弃了错误字节。 */
int HostCodec_Feed(HostCodec *p, uint8_t byte, uint32_t now);
#endif
#endif
