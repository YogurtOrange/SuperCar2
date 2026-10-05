/*
 * 整车控制模块：板级 IO -> 故障/保活保护 -> 整车初始化与协作调度。
 * main 只接入 CarControl_Init / CarControl_Poll；各时间参数单位 ms。
 * PB12/13/14 是主循环扫描的低有效输入；故障按位锁存，恢复由 car_motor 核验。
 */
#include "car_control.h"
#include "car_uart.h"
#include "car_motor.h"
#include "car_config.h"

/* ==================== 板级 IO：急停/报警输入、状态 LED 与独立看门狗 ==================== */
/* 再次配置板级 IO，以免 CubeMX 重新生成后丢设置；先给低有效 LED 高电平，再设输出。 */
static void board_init(void)
{
    GPIO_InitTypeDef gpio={0};
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin=GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
    gpio.Mode=GPIO_MODE_INPUT; gpio.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB,&gpio);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
    gpio.Pin=GPIO_PIN_13; gpio.Mode=GPIO_MODE_OUTPUT_PP;
    gpio.Pull=GPIO_NOPULL; gpio.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC,&gpio);
#if CAR_USE_IWDG
    /* 独立看门狗：PR=4 对应 64 分频，RLR=624 对应 625 个分频时钟。
       超时约为 64*(624+1)/f_LSI 秒，标称约 1 s，实际以板上 LSI 为准。
       需要改超时应核对这两处寄存器；CAR_USE_IWDG 只是开关。
       调试暂停时冻结；复位后重新经过启动停止/失能，不能替代硬件断电急停。 */
    DBGMCU->CR|=DBGMCU_CR_DBG_IWDG_STOP;
    IWDG->KR=0xCCCC;
    IWDG->KR=0x5555; IWDG->PR=4; IWDG->RLR=624;
    {
        uint32_t start=HAL_GetTick();
        while(IWDG->SR!=0) {
            if((uint32_t)(HAL_GetTick()-start)>100U) Error_Handler();
        }
    }
    IWDG->KR=0xAAAA;
#endif
}
/* 正常每 500 ms 翻转 LED，故障每 100 ms 翻转；这里为固定值，末尾每轮喂狗。 */
static void board_poll(uint32_t now, uint16_t faults)
{
    uint32_t period=faults ? 100U : 500U;
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,(now/period)&1U ? GPIO_PIN_SET : GPIO_PIN_RESET);
#if CAR_USE_IWDG
    IWDG->KR=0xAAAA;
#endif
}

/* ==================== 保护逻辑：保活超时、故障锁存与显式清除条件 ==================== */
/* faults 为位掩码而非单个错误编号；last_host 只记录协议层接受的有效保活。 */
static uint16_t faults;
static uint32_t last_host;
uint8_t CarControl_InputsReleased(void)
{
#if CAR_USE_ESTOP_PIN
    if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==GPIO_PIN_RESET) return 0;
#endif
#if CAR_USE_FAULT_PINS
    if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)==GPIO_PIN_RESET ||
       HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET) return 0;
#endif
    return 1;
}
static void safety_init(uint32_t now) { faults=0; last_host=now; }
void CarControl_Heartbeat(uint32_t now) { last_host=now; }
/* 只增加故障位，不会被新的运动/心跳清除；电机调度随后执行优先停止。 */
void CarControl_Trip(uint16_t f) { faults|=f; }
uint16_t CarControl_Faults(void) { return faults; }
/* 先读物理输入，再检查活动状态的主机超时；已使能零速/STOP 仍属活动状态。 */
static void safety_poll(uint32_t now, uint8_t active)
{
#if CAR_USE_ESTOP_PIN
    if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==GPIO_PIN_RESET) CarControl_Trip(FAULT_ESTOP);
#endif
#if CAR_USE_FAULT_PINS
    if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)==GPIO_PIN_RESET ||
       HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET) CarControl_Trip(FAULT_DRIVER);
#endif
    if(active && (uint32_t)(now-last_host)>=CAR_HOST_TIMEOUT_MS) CarControl_Trip(FAULT_HOST_TIMEOUT);
}
/* 仅在 car_motor 已验证失能/零速/在线/配置/新鲜反馈后调用，且物理输入必须释放。 */
uint8_t CarControl_Clear(uint8_t safe, uint32_t now)
{
    if(!safe || !CarControl_InputsReleased()) return 0;
    faults=0; last_host=now; return 1;
}

/* ==================== 整车入口：固定初始化顺序与协作式主循环 ==================== */
static uint32_t last_report;
/* 初始化顺序保持：板级->串口->保护->主机协议->电机->遥测计时。 */
void CarControl_Init(UART_HandleTypeDef *motor, UART_HandleTypeDef *host)
{
    uint32_t now=HAL_GetTick();
    board_init(); CarUart_Init(motor,host);
    safety_init(now); CarUart_HostInit(); CarMotor_Init(now); last_report=now;
}
/* 每轮尽快执行，不能加 HAL_Delay 等 ACK；串口维护与命令处理分开以保留两次保护检查。 */
void CarControl_Poll(void)
{
    uint32_t now=HAL_GetTick();
    CarUart_Poll();
    /* 先检查物理急停，再处理命令；命令可能改变活动状态，因此再次检查。 */
    safety_poll(now,CarMotor_Active());
    CarUart_HostPoll(now);
    safety_poll(now,CarMotor_Active());
    CarMotor_Poll(now);
    if((uint32_t)(now-last_report)>=CAR_TELEMETRY_PERIOD_MS) {
        last_report=now; CarUart_HostReport(now);
    }
    board_poll(now,CarControl_Faults());
}
