#include "port.h"
#include "comm.h"
#include "usart.h"
#include "rng.h"
#include "main.h"
#include <string.h>

#define RX_CAPACITY 512U

static uint8_t rx_byte;
static uint8_t rx_data[RX_CAPACITY];
static uint32_t rx_tick[RX_CAPACITY];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint8_t rx_error;
static uint8_t tx_data[FRAME_WIRE_MAX];
static uint8_t imu_byte, imu_data[RX_CAPACITY];
static uint32_t imu_tick[RX_CAPACITY];
static volatile uint16_t imu_head, imu_tail;
static volatile uint8_t imu_error;
static uint32_t imu_source = 1;
static ImuParser imu;
static volatile PortDiag diagnostics[2];

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
    Imu_Init(&imu, imu_source);
    /* 外部串口先于本机启动时可能已有ORE/FE；仅首次挂接前丢弃残留字节。
     * 开始接收后所有错误仍记录、失效会话并停止，不清除运行期锁存。 */
    diagnostics[0].startup_flags = huart1.Instance->SR & (USART_SR_ORE | USART_SR_NE | USART_SR_FE);
    diagnostics[1].startup_flags = huart2.Instance->SR & (USART_SR_ORE | USART_SR_NE | USART_SR_FE);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK)
    {
        rx_error = 1;
        diagnostics[0].restart_errors++;
    }
    if (HAL_UART_Receive_IT(&huart2, &imu_byte, 1) != HAL_OK)
    {
        imu_error = 1;
        diagnostics[1].restart_errors++;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint16_t next;
    /* 复用旧工程的错误交接：坏字节不入队，不在HAL错误分支内重挂接。
     * Receive_IT会清ErrorCode；此处重挂接会掩盖错误并干扰HAL终止接收。 */
    if (HAL_UART_GetError(uart) != HAL_UART_ERROR_NONE)
    {
        return;
    }
    if (uart == &huart2)
    {
        diagnostics[1].rx_bytes++;
        next = (uint16_t)((imu_head + 1U) % RX_CAPACITY);
        if (next == imu_tail)
        {
            imu_error = 1;
            diagnostics[1].overflows++;
        }
        else
        {
            imu_data[imu_head] = imu_byte;
            imu_tick[imu_head] = HAL_GetTick();
            __DMB();
            imu_head = next;
        }
        if (HAL_UART_Receive_IT(&huart2, &imu_byte, 1) != HAL_OK)
        {
            imu_error = 1;
            diagnostics[1].restart_errors++;
        }
        return;
    }
    if (uart != &huart1)
    {
        return;
    }
    diagnostics[0].rx_bytes++;
    next = (uint16_t)((rx_head + 1U) % RX_CAPACITY);
    if (next == rx_tail)
    {
        rx_error = 1;
        diagnostics[0].overflows++;
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
        diagnostics[0].restart_errors++;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    unsigned index;
    if (uart == &huart1 || uart == &huart2)
    {
        index = uart == &huart1 ? 0U : 1U;
        if (!diagnostics[index].errors)
        {
            diagnostics[index].first_error_code = uart->ErrorCode;
            diagnostics[index].first_error_ms = HAL_GetTick();
        }
        diagnostics[index].errors++;
        diagnostics[index].last_error_code = uart->ErrorCode;
        diagnostics[index].last_error_ms = HAL_GetTick();
    }
    if (uart == &huart1)
    {
        rx_error = 1;
    }
    if (uart == &huart2)
    {
        imu_error = 1;
    }
}

uint16_t Port_Diag(uint8_t *data)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    memcpy(data, (const void *)diagnostics, sizeof(diagnostics));
    __set_PRIMASK(mask);
    return sizeof(diagnostics);
}

int Port_Imu(ImuSample *sample)
{
    while (Imu_Read(&imu, sample))
    {
        if (Imu_Valid(&imu, sample, HAL_GetTick()))
        {
            return 1;
        }
    }
    return 0;
}

void Port_Run(void)
{
    unsigned count = 0;
    uint16_t size;
    uint16_t tail;
    uint8_t byte;
    uint32_t tick;
    uint32_t irq;
    if (imu_error)
    {
        irq = __get_PRIMASK();
        __disable_irq();
        imu_tail = imu_head;
        imu_error = 0;
        __set_PRIMASK(irq);
        Imu_Loss(&imu, ++imu_source);
        Comm_Fault(8U);
        (void)HAL_UART_AbortReceive(&huart2);
        if (HAL_UART_Receive_IT(&huart2, &imu_byte, 1) != HAL_OK)
        {
            imu_error = 1;
        }
    }
    while (imu_tail != imu_head && count++ < RX_CAPACITY)
    {
        tail = imu_tail;
        __DMB();
        byte = imu_data[tail];
        tick = imu_tick[tail];
        __DMB();
        imu_tail = (uint16_t)((tail + 1U) % RX_CAPACITY);
        Imu_Feed(&imu, byte, tick, HAL_GetTick(), imu_source);
    }
    if (imu_tail == imu_head)
    {
        Imu_Tick(&imu, HAL_GetTick());
    }
    count = 0;
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
