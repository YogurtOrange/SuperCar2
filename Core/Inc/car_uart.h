#ifndef CAR_UART_H
#define CAR_UART_H
#include "main.h"
#include "car_config.h"

/* 两路 UART 的非阻塞收发；调用者不得在中断里解析控制命令。 */
typedef enum { UART_MOTOR=0, UART_HOST=1 } UartPort;
/* motor 接 USART1、host 接 USART2；启动逐字节中断接收。 */
void CarUart_Init(UART_HandleTypeDef *motor, UART_HandleTypeDef *host);
/* 维护 TX 队列、接收重挂与卡死检测；不解析主机命令。 */
void CarUart_Poll(void);
/* 返回 1 并写出一个字节；返回 0 表示当前接收队列为空。 */
uint8_t CarUart_Read(UartPort port, uint8_t *byte);
/* 复制至持久缓冲并启动非阻塞 TX，1=已提交，0=忙/帧间隔未到/长度越界/HAL 失败。
 * 长度最多 16 字节；成功提交不等于驱动器已应答，事务由 car_motor 跟踪。 */
uint8_t CarUart_SendMotor(const uint8_t *data, uint8_t length);
uint8_t CarUart_MotorTxBusy(void);
/* 原子读取并清除串口错误/溢出/TX 卡死标志；调用方负责锁存整车故障。 */
uint8_t CarUart_TakeRxFault(UartPort port);
/* 原子丢弃已排队 RX 字节；不改变正在发送的帧。 */
void CarUart_FlushRx(UartPort port);

/* 主循环分开调用维护与命令处理，保证保护检查先于命令执行。 */
void CarUart_HostInit(void);
/* 消费接收字节，处理 ASCII/二进制命令；应放在两次保护检查之间。 */
void CarUart_HostPoll(uint32_t now);
/* 把当前缓存反馈排入 TX；不主动等待新电机采样。队列满时允许丢弃回报。 */
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
