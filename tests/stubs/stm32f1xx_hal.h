#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef struct { uint8_t id; } UART_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
typedef struct { volatile uint32_t KR, PR, RLR, SR; } TestIwdg;
typedef struct { volatile uint32_t CR; } TestDbgMcu;
extern TestIwdg test_iwdg;
extern TestDbgMcu test_dbgmcu;
#define IWDG (&test_iwdg)
#define DBGMCU (&test_dbgmcu)
#define DBGMCU_CR_DBG_IWDG_STOP (1U<<8)
#define GPIOB ((void *)2)
#define GPIOC ((void *)3)
#define GPIO_PIN_12 4096U
#define GPIO_PIN_13 8192U
#define GPIO_PIN_14 16384U
#define GPIO_MODE_INPUT 0U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_PULLUP 1U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_LOW 2U
extern uint8_t test_gpio_b_clock, test_gpio_c_clock;
#define __HAL_RCC_GPIOB_CLK_ENABLE() (test_gpio_b_clock=1)
#define __HAL_RCC_GPIOC_CLK_ENABLE() (test_gpio_c_clock=1)
#define __DMB() ((void)0)
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
GPIO_PinState HAL_GPIO_ReadPin(void *, uint16_t);
void HAL_GPIO_Init(void *, GPIO_InitTypeDef *);
void HAL_GPIO_WritePin(void *, uint16_t, GPIO_PinState);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
#endif
