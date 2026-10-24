#include "port.h"
#include "comm.h"
#include "usart.h"
#include "rng.h"
#include "main.h"

#define RX_CAPACITY 512U

static uint8_t rx_byte;
static uint8_t rx_data[RX_CAPACITY];
static uint32_t rx_tick[RX_CAPACITY];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint8_t rx_error;
static uint8_t tx_data[FRAME_WIRE_MAX];

void Port_Init(void)
{
    uint32_t low = 0;
    uint32_t high = 0;
    uint64_t boot = 0;
    /* 仅启动时取随机启动身份；失败则禁止建立会话，不伪造启动号。 */
    if (HAL_RNG_GenerateRandomNumber(&hrng, &low) == HAL_OK &&
        HAL_RNG_GenerateRandomNumber(&hrng, &high) == HAL_OK)
    {
        boot = ((uint64_t)high << 32) | low;
    }
    Comm_Init(boot, HAL_GetTick());
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK)
    {
        rx_error = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint16_t next;
    if (uart != &huart1)
    {
        return;
    }
    next = (uint16_t)((rx_head + 1U) % RX_CAPACITY);
    if (next == rx_tail)
    {
        rx_error = 1;
    }
    else
    {
        rx_data[rx_head] = rx_byte;
        rx_tick[rx_head] = HAL_GetTick();
        __DMB();
        rx_head = next;
    }
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK)
    {
        rx_error = 1;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1)
    {
        rx_error = 1;
    }
}

void Port_Run(void)
{
    unsigned count = 0;
    uint16_t size;
    uint16_t tail;
    uint8_t byte;
    uint32_t tick;
    uint32_t irq;
    Comm_Tick(HAL_GetTick(), HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) == GPIO_PIN_RESET);
    if (rx_error)
    {
        irq = __get_PRIMASK();
        __disable_irq();
        rx_tail = rx_head;
        rx_error = 0;
        __set_PRIMASK(irq);
        Comm_Fault(2U);
        (void)HAL_UART_AbortReceive(&huart1);
        if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK)
        {
            rx_error = 1;
        }
    }
    while (rx_tail != rx_head && count++ < RX_CAPACITY)
    {
        tail = rx_tail;
        __DMB();
        byte = rx_data[tail];
        tick = rx_tick[tail];
        __DMB();
        rx_tail = (uint16_t)((tail + 1U) % RX_CAPACITY);
        Comm_Feed(byte, tick, HAL_GetTick());
    }
    /* 发送缓冲直到中断发送结束才复用；普通队列满不占停止应答槽。 */
    if (huart1.gState == HAL_UART_STATE_READY)
    {
        size = Comm_Read(tx_data);
        if (size && HAL_UART_Transmit_IT(&huart1, tx_data, size) != HAL_OK)
        {
            Comm_Fault(4U);
        }
    }
}
