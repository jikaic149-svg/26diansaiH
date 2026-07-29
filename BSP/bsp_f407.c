/* STM32F407VET6 BSP：时钟、KEY1、USART1(板间通信)、USART3(ESP8266)、CAN1(QD4310)。
 *
 * 硬件映射（来自开发板例程与 QD4310 手册）：
 *   KEY1    PA0 input, pull-down, active high
 *   USART1  PA9 TX / PA10 RX  115200 8N1  APB2 84MHz   DMA2 S7/S2 CH4
 *   USART3  PB10 TX / PB11 RX 115200 8N1  APB1 42MHz   ESP8266
 *   CAN1    PB9 TX / PB8 RX   1Mbps       APB1 42MHz   QD4310 总线
 *           BTR: 预分频6, BS1=5TQ, BS2=1TQ, SJW=1TQ -> 42M/(6*7)=1M
 *
 * 中断优先级统一为 6（NVIC 4bit，数值 >= configMAX_SYSCALL_INTERRUPT_PRIORITY
 * 对应的 5，允许在 ISR 内使用 FromISR API；本 BSP 的 ISR 不调用 RTOS）。
 */

#include "bsp_init.h"
#include "bsp_key.h"
#include "bsp_uart.h"
#include "bsp_can.h"

#include <string.h>

#include "stm32f4xx.h"

#define BSP_APB1_CLOCK_HZ      42000000UL
#define BSP_APB2_CLOCK_HZ      84000000UL
#define BSP_UART_BAUD          115200UL
#define BSP_UART_RING_SIZE     512U
#define BSP_UART_RING_MASK     (BSP_UART_RING_SIZE - 1U)
#define BSP_UART_DMA_RX_SIZE   64U
#define BSP_UART_IRQ_PRIORITY  6U
#define BSP_DMA_IRQ_PRIORITY   6U
#define BSP_CAN_IRQ_PRIORITY   6U
#define BSP_CAN_RX_QUEUE_SIZE  16U
#define BSP_CLOCK_POLL_LIMIT   4000000UL
#define BSP_UART_ERROR_MASK    (USART_SR_PE | USART_SR_FE | USART_SR_NE | \
                                USART_SR_ORE)

_Static_assert((BSP_UART_RING_SIZE & BSP_UART_RING_MASK) == 0U,
               "ring size must be a power of two");
_Static_assert((BSP_UART_DMA_RX_SIZE & (BSP_UART_DMA_RX_SIZE - 1U)) == 0U,
               "DMA RX size must be a power of two");

/* ------------------------------------------------------------------ */
/* 无锁单生产者/单消费者字节环（ISR 单端生产或消费）                       */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t data[BSP_UART_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} uart_ring_t;

typedef struct {
    uint8_t rx_stage[BSP_UART_DMA_RX_SIZE];
    volatile uint16_t rx_committed;
    volatile uint16_t tx_length;
} uart_dma_state_t;

static bool ring_push(uart_ring_t *ring, uint8_t value)
{
    const uint16_t next = (uint16_t)((ring->head + 1U) & BSP_UART_RING_MASK);

    if (next == ring->tail) {
        return false;
    }
    ring->data[ring->head] = value;
    ring->head = next;
    return true;
}

static bool ring_pop(uart_ring_t *ring, uint8_t *value)
{
    if (ring->tail == ring->head) {
        return false;
    }
    *value = ring->data[ring->tail];
    ring->tail = (uint16_t)((ring->tail + 1U) & BSP_UART_RING_MASK);
    return true;
}

static uint16_t ring_used(const uart_ring_t *ring)
{
    return (uint16_t)((ring->head - ring->tail) & BSP_UART_RING_MASK);
}

static uint16_t ring_free(const uart_ring_t *ring)
{
    return (uint16_t)(BSP_UART_RING_MASK - ring_used(ring));
}

static void counter_increment_saturating(volatile uint32_t *counter)
{
    if (*counter < UINT32_MAX) {
        ++*counter;
    }
}

static void ring_reset(uart_ring_t *ring)
{
    ring->head = 0U;
    ring->tail = 0U;
}

/* ------------------------------------------------------------------ */
/* UART 资源                                                            */
/* ------------------------------------------------------------------ */

struct BSP_UART_Resource {
    USART_TypeDef *instance;
    IRQn_Type irq;
    uart_ring_t *tx_ring;
    uart_ring_t *rx_ring;
    volatile uint32_t *sw_overflow;
    volatile uint32_t *hw_overrun;
    volatile uint32_t *framing;
    volatile uint32_t *parity;
    volatile uint32_t *dma_error;
    volatile uint32_t *sticky;
    uart_dma_state_t *dma;
    bool uses_dma;
};

static uart_ring_t s_uart1_tx, s_uart1_rx;
static uart_ring_t s_uart3_tx, s_uart3_rx;
static uart_dma_state_t s_uart1_dma;
static volatile uint32_t s_uart1_err[6];
static volatile uint32_t s_uart3_err[6];

static const struct BSP_UART_Resource s_uart_resources[] = {
    {
        .instance = USART1, .irq = USART1_IRQn,
        .tx_ring = &s_uart1_tx, .rx_ring = &s_uart1_rx,
        .sw_overflow = &s_uart1_err[0], .hw_overrun = &s_uart1_err[1],
        .framing = &s_uart1_err[2], .parity = &s_uart1_err[3],
        .dma_error = &s_uart1_err[4], .sticky = &s_uart1_err[5],
        .dma = &s_uart1_dma, .uses_dma = true,
    },
    {
        .instance = USART3, .irq = USART3_IRQn,
        .tx_ring = &s_uart3_tx, .rx_ring = &s_uart3_rx,
        .sw_overflow = &s_uart3_err[0], .hw_overrun = &s_uart3_err[1],
        .framing = &s_uart3_err[2], .parity = &s_uart3_err[3],
        .dma_error = &s_uart3_err[4], .sticky = &s_uart3_err[5],
        .dma = NULL, .uses_dma = false,
    },
};

#define BSP_UART_TRANSPORT_COUNT \
    (sizeof(s_uart_resources) / sizeof(s_uart_resources[0]))

static bool uart_handle_valid(BSP_UART_Handle uart)
{
    for (size_t index = 0U; index < BSP_UART_TRANSPORT_COUNT; ++index) {
        if (uart == &s_uart_resources[index]) {
            return true;
        }
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* 时钟：HSE 25MHz -> PLL -> 168MHz，APB1/4，APB2/2                     */
/* ------------------------------------------------------------------ */

static bool wait_flag(volatile const uint32_t *reg, uint32_t mask, bool set)
{
    uint32_t remaining = BSP_CLOCK_POLL_LIMIT;

    while (remaining > 0U) {
        if (((*reg & mask) != 0U) == set) {
            return true;
        }
        --remaining;
    }
    return false;
}

static void key_init(void)
{
    /* KEY1 is a board-level active-high button on PA0. */
    GPIOA->MODER &= ~(UINT32_C(0x3) << (0U * 2U));
    GPIOA->PUPDR = (GPIOA->PUPDR &
                    ~(UINT32_C(0x3) << (0U * 2U))) |
                   (UINT32_C(0x2) << (0U * 2U));
}

bool BSP_Key_IsPressed(void)
{
    return (GPIOA->IDR & (UINT32_C(1) << 0U)) != 0U;
}

static bool clock_init(void)
{
    RCC->CR |= RCC_CR_HSEON;
    if (!wait_flag(&RCC->CR, RCC_CR_HSERDY, true)) {
        return false;
    }

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN |
                 FLASH_ACR_ICEN | FLASH_ACR_DCEN;

    RCC->CFGR = (RCC->CFGR &
                 ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
                RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 |
                RCC_CFGR_PPRE2_DIV2;

    /* PLLM=25 PLLN=336 PLLP=2(00) PLLQ=7: VCO=336MHz, SYSCLK=168MHz */
    RCC->PLLCFGR = 25U | (336U << RCC_PLLCFGR_PLLN_Pos) |
                   RCC_PLLCFGR_PLLSRC_HSE |
                   (7U << RCC_PLLCFGR_PLLQ_Pos);
    RCC->CR |= RCC_CR_PLLON;
    if (!wait_flag(&RCC->CR, RCC_CR_PLLRDY, true)) {
        return false;
    }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    if (!wait_flag(&RCC->CFGR, RCC_CFGR_SWS_PLL, true)) {
        return false;
    }

    SystemCoreClock = 168000000UL;
    return true;
}

/* ------------------------------------------------------------------ */
/* GPIO 复用配置                                                        */
/* ------------------------------------------------------------------ */

static void gpio_af_init(
    GPIO_TypeDef *gpio, uint8_t pin, uint8_t alternate)
{
    const uint32_t mode_pos = (uint32_t)pin * 2U;
    const uint32_t afr_index = (pin >= 8U) ? 1U : 0U;
    const uint32_t afr_pos = ((uint32_t)pin % 8U) * 4U;

    gpio->MODER = (gpio->MODER & ~(3UL << mode_pos)) | (2UL << mode_pos);
    gpio->OTYPER &= ~(1UL << pin);
    gpio->OSPEEDR |= 3UL << mode_pos;
    gpio->PUPDR = (gpio->PUPDR & ~(3UL << mode_pos)) | (1UL << mode_pos);
    gpio->AFR[afr_index] =
        (gpio->AFR[afr_index] & ~(0xFUL << afr_pos)) |
        ((uint32_t)alternate << afr_pos);
}

/* ------------------------------------------------------------------ */
/* UART 初始化与中断：USART1 使用 DMA2，USART3 保留字节中断后端             */
/* ------------------------------------------------------------------ */

static void uart_periph_init(
    USART_TypeDef *instance, uint32_t peripheral_clock_hz, bool uses_dma)
{
    instance->BRR =
        (peripheral_clock_hz + (BSP_UART_BAUD / 2U)) / BSP_UART_BAUD;
    instance->CR2 = 0U;
    instance->CR3 = uses_dma ?
        (USART_CR3_DMAR | USART_CR3_DMAT | USART_CR3_EIE) : 0U;
    instance->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE |
        (uses_dma ? USART_CR1_IDLEIE : USART_CR1_RXNEIE);
}

static bool uart_dma_start_rx(const struct BSP_UART_Resource *uart)
{
    DMA_Stream_TypeDef *stream = DMA2_Stream2;

    if ((uart == NULL) || !uart->uses_dma ||
        ((stream->CR & DMA_SxCR_EN) != 0U)) {
        return false;
    }
    DMA2->LIFCR = DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 |
        DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;
    uart->dma->rx_committed = 0U;
    stream->PAR = (uint32_t)&uart->instance->DR;
    stream->M0AR = (uint32_t)uart->dma->rx_stage;
    stream->NDTR = BSP_UART_DMA_RX_SIZE;
    stream->FCR = 0U;
    stream->CR = (4UL << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC |
        DMA_SxCR_CIRC | DMA_SxCR_TCIE | DMA_SxCR_HTIE |
        DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    stream->CR |= DMA_SxCR_EN;
    return true;
}

/* Called only with interrupts masked or from an ISR. DMA half/full IRQs keep
 * the uncommitted window below half the staging buffer; UART IDLE makes short
 * heartbeat frames visible without waiting for the next half-buffer event. */
static void uart_dma_commit_rx(const struct BSP_UART_Resource *uart)
{
    uint32_t remaining = DMA2_Stream2->NDTR;
    uint16_t position;

    if (remaining > BSP_UART_DMA_RX_SIZE) {
        counter_increment_saturating(uart->dma_error);
        *uart->sticky |= BSP_UART_RX_ERROR_DMA;
        return;
    }
    position = (uint16_t)(BSP_UART_DMA_RX_SIZE - remaining);
    if (position == BSP_UART_DMA_RX_SIZE) {
        position = 0U;
    }
    while (uart->dma->rx_committed != position) {
        const uint16_t index = uart->dma->rx_committed;

        if (!ring_push(uart->rx_ring, uart->dma->rx_stage[index])) {
            counter_increment_saturating(uart->sw_overflow);
            *uart->sticky |= BSP_UART_RX_ERROR_SOFTWARE_OVERFLOW;
        }
        uart->dma->rx_committed =
            (uint16_t)((index + 1U) & (BSP_UART_DMA_RX_SIZE - 1U));
    }
}

static void uart_dma_start_tx(const struct BSP_UART_Resource *uart)
{
    DMA_Stream_TypeDef *stream = DMA2_Stream7;
    uint16_t length;

    if ((uart == NULL) || !uart->uses_dma ||
        (uart->dma->tx_length != 0U) ||
        (uart->tx_ring->tail == uart->tx_ring->head) ||
        ((stream->CR & DMA_SxCR_EN) != 0U)) {
        return;
    }
    length = (uart->tx_ring->head > uart->tx_ring->tail) ?
        (uint16_t)(uart->tx_ring->head - uart->tx_ring->tail) :
        (uint16_t)(BSP_UART_RING_SIZE - uart->tx_ring->tail);
    DMA2->HIFCR = DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 |
        DMA_HIFCR_CTEIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7;
    stream->PAR = (uint32_t)&uart->instance->DR;
    stream->M0AR = (uint32_t)&uart->tx_ring->data[uart->tx_ring->tail];
    stream->NDTR = length;
    stream->FCR = 0U;
    stream->CR = (4UL << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_DIR_0 |
        DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    uart->dma->tx_length = length;
    stream->CR |= DMA_SxCR_EN;
}

static void uart_dma_complete_tx(const struct BSP_UART_Resource *uart)
{
    const uint16_t completed = uart->dma->tx_length;

    if (completed == 0U) {
        counter_increment_saturating(uart->dma_error);
        *uart->sticky |= BSP_UART_RX_ERROR_DMA;
        return;
    }
    uart->tx_ring->tail =
        (uint16_t)((uart->tx_ring->tail + completed) & BSP_UART_RING_MASK);
    uart->dma->tx_length = 0U;
    uart_dma_start_tx(uart);
}

static void uart_dma_rx_error(const struct BSP_UART_Resource *uart)
{
    counter_increment_saturating(uart->dma_error);
    *uart->sticky |= BSP_UART_RX_ERROR_DMA;
    DMA2_Stream2->CR &= ~DMA_SxCR_EN;
    if ((DMA2_Stream2->CR & DMA_SxCR_EN) == 0U) {
        (void)uart_dma_start_rx(uart);
    }
}

static void uart_dma_tx_error(const struct BSP_UART_Resource *uart)
{
    counter_increment_saturating(uart->dma_error);
    *uart->sticky |= BSP_UART_RX_ERROR_DMA;
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    uart->tx_ring->tail = uart->tx_ring->head;
    uart->dma->tx_length = 0U;
}

static void uart_irq_common(const struct BSP_UART_Resource *uart)
{
    USART_TypeDef *instance = uart->instance;
    const uint32_t status = instance->SR;
    uint8_t value;

    if ((status & (USART_SR_RXNE | BSP_UART_ERROR_MASK)) != 0U) {
        value = (uint8_t)instance->DR;
        if ((status & USART_SR_ORE) != 0U) {
            counter_increment_saturating(uart->hw_overrun);
            *uart->sticky |= BSP_UART_RX_ERROR_HARDWARE_OVERRUN;
        }
        if ((status & USART_SR_FE) != 0U) {
            counter_increment_saturating(uart->framing);
            *uart->sticky |= BSP_UART_RX_ERROR_FRAMING;
        }
        if ((status & USART_SR_PE) != 0U) {
            counter_increment_saturating(uart->parity);
            *uart->sticky |= BSP_UART_RX_ERROR_PARITY;
        }
        if ((status & USART_SR_RXNE) != 0U) {
            if (!ring_push(uart->rx_ring, value)) {
                counter_increment_saturating(uart->sw_overflow);
                *uart->sticky |= BSP_UART_RX_ERROR_SOFTWARE_OVERFLOW;
            }
        }
    }

    if (((status & USART_SR_TXE) != 0U) &&
        ((instance->CR1 & USART_CR1_TXEIE) != 0U)) {
        if (ring_pop(uart->tx_ring, &value)) {
            instance->DR = value;
        }
        else {
            instance->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

void USART1_IRQHandler(void)
{
    const struct BSP_UART_Resource *uart = &s_uart_resources[0];
    const uint32_t status = USART1->SR;

    if ((status & (USART_SR_IDLE | USART_SR_RXNE |
            BSP_UART_ERROR_MASK)) != 0U) {
        (void)USART1->DR;
        if ((status & USART_SR_ORE) != 0U) {
            counter_increment_saturating(uart->hw_overrun);
            *uart->sticky |= BSP_UART_RX_ERROR_HARDWARE_OVERRUN;
        }
        if ((status & USART_SR_FE) != 0U) {
            counter_increment_saturating(uart->framing);
            *uart->sticky |= BSP_UART_RX_ERROR_FRAMING;
        }
        if ((status & USART_SR_PE) != 0U) {
            counter_increment_saturating(uart->parity);
            *uart->sticky |= BSP_UART_RX_ERROR_PARITY;
        }
        uart_dma_commit_rx(uart);
    }
}

void USART3_IRQHandler(void)
{
    uart_irq_common(&s_uart_resources[1]);
}

void DMA2_Stream2_IRQHandler(void)
{
    const uint32_t status = DMA2->LISR;

    DMA2->LIFCR = DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 |
        DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;
    /* FIFO mode and FEIE are disabled, so FEIF2 is not a transfer failure.
     * Only enabled direct-mode and transfer-error sources are fatal. */
    if ((status & (DMA_LISR_DMEIF2 | DMA_LISR_TEIF2)) != 0U) {
        uart_dma_rx_error(&s_uart_resources[0]);
    }
    else if ((status & (DMA_LISR_HTIF2 | DMA_LISR_TCIF2)) != 0U) {
        uart_dma_commit_rx(&s_uart_resources[0]);
    }
}

void DMA2_Stream7_IRQHandler(void)
{
    const uint32_t status = DMA2->HISR;

    DMA2->HIFCR = DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 |
        DMA_HIFCR_CTEIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7;
    /* FIFO mode and FEIE are disabled, so FEIF7 may accompany a valid TC. */
    if ((status & (DMA_HISR_DMEIF7 | DMA_HISR_TEIF7)) != 0U) {
        uart_dma_tx_error(&s_uart_resources[0]);
    }
    else if ((status & DMA_HISR_TCIF7) != 0U) {
        uart_dma_complete_tx(&s_uart_resources[0]);
    }
}

/* ------------------------------------------------------------------ */
/* BSP_UART 接口实现                                                      */
/* ------------------------------------------------------------------ */

size_t BSP_UART_Count(void)
{
    return BSP_UART_TRANSPORT_COUNT;
}

BSP_UART_Handle BSP_UART_GetHandle(size_t index)
{
    return (index < BSP_UART_TRANSPORT_COUNT) ?
        &s_uart_resources[index] : NULL;
}

bool BSP_UART_GetInfo(BSP_UART_Handle uart, BSP_UART_Info *info)
{
    if (!uart_handle_valid(uart) || (info == NULL)) {
        return false;
    }
    *info = (BSP_UART_Info){
        .baud_rate = BSP_UART_BAUD,
        .supports_tx_dma = uart->uses_dma,
        .supports_rx_dma = uart->uses_dma,
        .supports_buffered_tx = true,
        .supports_buffered_rx = true,
        .tx_buffer_capacity = BSP_UART_RING_MASK,
        .rx_buffer_capacity = BSP_UART_RING_MASK,
    };
    return true;
}

BSP_UART_Result BSP_UART_WriteBuffered(
    BSP_UART_Handle uart, const uint8_t *data, size_t length)
{
    uint32_t primask;

    if (!uart_handle_valid(uart) ||
        ((data == NULL) && (length != 0U)) ||
        (length > BSP_UART_RING_MASK)) {
        return BSP_UART_RESULT_INVALID_ARGUMENT;
    }
    if (length == 0U) {
        return BSP_UART_RESULT_OK;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (ring_free(uart->tx_ring) < length) {
        if (primask == 0U) {
            __enable_irq();
        }
        return BSP_UART_RESULT_BUSY;
    }
    for (size_t index = 0U; index < length; ++index) {
        (void)ring_push(uart->tx_ring, data[index]);
    }
    if (uart->uses_dma) {
        uart_dma_start_tx(uart);
    }
    else {
        uart->instance->CR1 |= USART_CR1_TXEIE;
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return BSP_UART_RESULT_OK;
}

BSP_UART_Result BSP_UART_ReadAvailable(
    BSP_UART_Handle uart,
    uint8_t *data,
    size_t capacity,
    size_t *transferred)
{
    size_t count = 0U;
    uint8_t value;
    uint32_t primask;

    if (transferred != NULL) {
        *transferred = 0U;
    }
    if (!uart_handle_valid(uart) || (transferred == NULL) ||
        ((data == NULL) && (capacity != 0U))) {
        return BSP_UART_RESULT_INVALID_ARGUMENT;
    }
    if (uart->uses_dma) {
        primask = __get_PRIMASK();
        __disable_irq();
        uart_dma_commit_rx(uart);
        if (primask == 0U) {
            __enable_irq();
        }
    }
    while ((count < capacity) && ring_pop(uart->rx_ring, &value)) {
        data[count++] = value;
    }
    *transferred = count;
    return BSP_UART_RESULT_OK;
}

BSP_UART_Result BSP_UART_WriteBlocking(
    BSP_UART_Handle uart,
    const uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    size_t *transferred)
{
    (void)uart;
    (void)data;
    (void)length;
    (void)timeout_ms;
    if (transferred != NULL) {
        *transferred = 0U;
    }
    return BSP_UART_RESULT_UNSUPPORTED;
}

BSP_UART_Result BSP_UART_ReadBlocking(
    BSP_UART_Handle uart,
    uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    size_t *transferred)
{
    (void)uart;
    (void)data;
    (void)length;
    (void)timeout_ms;
    if (transferred != NULL) {
        *transferred = 0U;
    }
    return BSP_UART_RESULT_UNSUPPORTED;
}

bool BSP_UART_GetRxStatus(BSP_UART_Handle uart, BSP_UART_RxStatus *status)
{
    uint32_t primask;

    if (!uart_handle_valid(uart) || (status == NULL)) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (uart->uses_dma) {
        uart_dma_commit_rx(uart);
    }
    *status = (BSP_UART_RxStatus){
        .software_overflow_count = *uart->sw_overflow,
        .hardware_overrun_count = *uart->hw_overrun,
        .framing_error_count = *uart->framing,
        .parity_error_count = *uart->parity,
        .dma_error_count = *uart->dma_error,
        .sticky_errors = *uart->sticky,
        .buffered_bytes = ring_used(uart->rx_ring),
        .buffer_capacity = BSP_UART_RING_MASK,
    };
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}

bool BSP_UART_ClearRxStatus(BSP_UART_Handle uart)
{
    uint32_t primask;

    if (!uart_handle_valid(uart)) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *uart->sw_overflow = 0U;
    *uart->hw_overrun = 0U;
    *uart->framing = 0U;
    *uart->parity = 0U;
    *uart->dma_error = 0U;
    *uart->sticky = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}

bool BSP_UART_Recover(BSP_UART_Handle uart)
{
    uint32_t primask;
    bool recovered = true;

    if (!uart_handle_valid(uart)) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (uart->uses_dma) {
        uart->instance->CR3 &= ~(USART_CR3_DMAR | USART_CR3_DMAT);
        DMA2_Stream2->CR &= ~DMA_SxCR_EN;
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        recovered = ((DMA2_Stream2->CR & DMA_SxCR_EN) == 0U) &&
            ((DMA2_Stream7->CR & DMA_SxCR_EN) == 0U);
        DMA2->LIFCR = DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 |
            DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;
        DMA2->HIFCR = DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 |
            DMA_HIFCR_CTEIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7;
        uart->dma->rx_committed = 0U;
        uart->dma->tx_length = 0U;
    }
    else {
        uart->instance->CR1 &= ~USART_CR1_TXEIE;
    }
    ring_reset(uart->tx_ring);
    ring_reset(uart->rx_ring);
    (void)uart->instance->SR;
    (void)uart->instance->DR;
    *uart->sw_overflow = 0U;
    *uart->hw_overrun = 0U;
    *uart->framing = 0U;
    *uart->parity = 0U;
    *uart->dma_error = 0U;
    *uart->sticky = 0U;
    if (uart->uses_dma && recovered) {
        uart->instance->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
        recovered = uart_dma_start_rx(uart);
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return recovered;
}

BSP_UART_Result BSP_UART_StartWriteDMA(
    BSP_UART_Handle uart,
    const uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    BSP_UART_TransferCallback callback,
    void *context)
{
    (void)uart;
    (void)data;
    (void)length;
    (void)timeout_ms;
    (void)callback;
    (void)context;
    return BSP_UART_RESULT_UNSUPPORTED;
}

BSP_UART_Result BSP_UART_StartReadDMA(
    BSP_UART_Handle uart,
    uint8_t *data,
    size_t length,
    uint32_t timeout_ms,
    BSP_UART_TransferCallback callback,
    void *context)
{
    (void)uart;
    (void)data;
    (void)length;
    (void)timeout_ms;
    (void)callback;
    (void)context;
    return BSP_UART_RESULT_UNSUPPORTED;
}

bool BSP_UART_IsTxBusy(BSP_UART_Handle uart)
{
    return uart_handle_valid(uart) && (ring_used(uart->tx_ring) != 0U);
}

bool BSP_UART_IsRxBusy(BSP_UART_Handle uart)
{
    (void)uart;
    return false;
}

/* ------------------------------------------------------------------ */
/* CAN1：1Mbps，接收走 FIFO0 中断入软件队列，发送轮询空邮箱                */
/* ------------------------------------------------------------------ */

typedef struct {
    BSP_CAN_Frame frames[BSP_CAN_RX_QUEUE_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} can_rx_queue_t;

struct BSP_CAN_Resource {
    CAN_TypeDef *instance;
    can_rx_queue_t *rx_queue;
    volatile uint32_t *rx_dropped;
};

static can_rx_queue_t s_can1_rx_queue;
static volatile uint32_t s_can1_rx_dropped;

static const struct BSP_CAN_Resource s_can_resources[] = {
    {
        .instance = CAN1,
        .rx_queue = &s_can1_rx_queue,
        .rx_dropped = &s_can1_rx_dropped,
    },
};

static bool can_handle_valid(BSP_CAN_Handle can)
{
    return can == &s_can_resources[0];
}

static bool can_init(void)
{
    /* PB8=CAN1_RX PB9=CAN1_TX AF9 */
    gpio_af_init(GPIOB, 8U, 9U);
    gpio_af_init(GPIOB, 9U, 9U);

    RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;

    CAN1->MCR = CAN_MCR_INRQ;
    if (!wait_flag(&CAN1->MSR, CAN_MSR_INAK, true)) {
        return false;
    }

    /* 42MHz/6/(1+5+1)=1Mbps，采样点 (1+5)/7≈85.7%，SJW=1TQ。
     * 自动重传保持开启（QD4310 应答式协议依赖帧到达）。 */
    CAN1->BTR = (4UL << CAN_BTR_TS1_Pos) | (0UL << CAN_BTR_TS2_Pos) | 5UL;

    /* 过滤器0：32bit 掩码模式全通，入 FIFO0。 */
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FA1R &= ~CAN_FA1R_FACT0;
    CAN1->FM1R &= ~CAN_FM1R_FBM0;
    CAN1->FS1R |= CAN_FS1R_FSC0;
    CAN1->FFA1R &= ~CAN_FFA1R_FFA0;
    CAN1->sFilterRegister[0].FR1 = 0U;
    CAN1->sFilterRegister[0].FR2 = 0U;
    CAN1->FA1R |= CAN_FA1R_FACT0;
    CAN1->FMR &= ~CAN_FMR_FINIT;

    CAN1->MCR &= ~CAN_MCR_INRQ;
    if (!wait_flag(&CAN1->MSR, CAN_MSR_INAK, false)) {
        return false;
    }

    CAN1->IER |= CAN_IER_FMPIE0;
    NVIC_SetPriority(CAN1_RX0_IRQn, BSP_CAN_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    return true;
}

void CAN1_RX0_IRQHandler(void)
{
    can_rx_queue_t *queue = &s_can1_rx_queue;

    while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0U) {
        const uint16_t next =
            (uint16_t)((queue->head + 1U) % BSP_CAN_RX_QUEUE_SIZE);
        const uint32_t rir = CAN1->sFIFOMailBox[0].RIR;
        const uint32_t rdtr = CAN1->sFIFOMailBox[0].RDTR;
        const uint32_t low = CAN1->sFIFOMailBox[0].RDLR;
        const uint32_t high = CAN1->sFIFOMailBox[0].RDHR;

        if (next == queue->tail) {
            ++s_can1_rx_dropped;
        }
        else {
            BSP_CAN_Frame *frame = &queue->frames[queue->head];

            memset(frame, 0, sizeof(*frame));
            if ((rir & CAN_RI0R_IDE) != 0U) {
                frame->identifier = (rir >> 3U) & 0x1FFFFFFFUL;
                frame->extended_identifier = true;
            }
            else {
                frame->identifier = (rir >> 21U) & 0x7FFUL;
            }
            frame->data_length = rdtr & 0x0FUL;
            if (frame->data_length > BSP_CAN_CLASSIC_MAX_DATA_LENGTH) {
                frame->data_length = BSP_CAN_CLASSIC_MAX_DATA_LENGTH;
            }
            for (size_t index = 0U; index < frame->data_length; ++index) {
                frame->data[index] = (index < 4U) ?
                    (uint8_t)(low >> (8U * index)) :
                    (uint8_t)(high >> (8U * (index - 4U)));
            }
            queue->head = next;
        }
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    }
}

size_t BSP_CAN_Count(void)
{
    return 1U;
}

BSP_CAN_Handle BSP_CAN_GetHandle(size_t index)
{
    return (index == 0U) ? &s_can_resources[0] : NULL;
}

bool BSP_CAN_GetInfo(BSP_CAN_Handle can, BSP_CAN_Info *info)
{
    if (!can_handle_valid(can) || (info == NULL)) {
        return false;
    }
    *info = (BSP_CAN_Info){
        .nominal_bit_rate = 1000000UL,
        .data_bit_rate = 1000000UL,
        .supports_fd = false,
        .supports_bit_rate_switch = false,
    };
    return true;
}

BSP_CAN_Result BSP_CAN_Send(
    BSP_CAN_Handle can, const BSP_CAN_Frame *frame, uint32_t timeout_ms)
{
    uint8_t mailbox;
    uint32_t low = 0U;
    uint32_t high = 0U;
    /* SysTick 1ms 由 FreeRTOS 驱动；这里用粗略循环计数换算超时，
     * 每毫秒约 168000/8 次轮询迭代，只作为上限保护。 */
    uint32_t spins = (timeout_ms == 0U) ? 1U : (timeout_ms * 21000U);

    if (!can_handle_valid(can) || (frame == NULL) ||
        frame->fd_format || frame->bit_rate_switch ||
        (frame->data_length > BSP_CAN_CLASSIC_MAX_DATA_LENGTH) ||
        (!frame->extended_identifier && (frame->identifier > 0x7FFUL)) ||
        (frame->extended_identifier &&
         (frame->identifier > 0x1FFFFFFFUL))) {
        return BSP_CAN_RESULT_INVALID_ARGUMENT;
    }
    if (BSP_CAN_IsBusOff(can)) {
        return BSP_CAN_RESULT_BUS_OFF;
    }

    for (;;) {
        if ((CAN1->TSR & CAN_TSR_TME0) != 0U) {
            mailbox = 0U;
            break;
        }
        if ((CAN1->TSR & CAN_TSR_TME1) != 0U) {
            mailbox = 1U;
            break;
        }
        if ((CAN1->TSR & CAN_TSR_TME2) != 0U) {
            mailbox = 2U;
            break;
        }
        if (spins == 0U) {
            return BSP_CAN_RESULT_TIMEOUT;
        }
        --spins;
    }

    for (size_t index = 0U; index < frame->data_length; ++index) {
        if (index < 4U) {
            low |= (uint32_t)frame->data[index] << (8U * index);
        }
        else {
            high |= (uint32_t)frame->data[index] << (8U * (index - 4U));
        }
    }

    CAN1->TSR = CAN_TSR_RQCP0 << (8U * mailbox);
    CAN1->sTxMailBox[mailbox].TDTR = frame->data_length;
    CAN1->sTxMailBox[mailbox].TDLR = low;
    CAN1->sTxMailBox[mailbox].TDHR = high;
    CAN1->sTxMailBox[mailbox].TIR = frame->extended_identifier ?
        ((frame->identifier << 3U) | CAN_TI0R_IDE | CAN_TI0R_TXRQ) :
        ((frame->identifier << 21U) | CAN_TI0R_TXRQ);
    return BSP_CAN_RESULT_OK;
}

BSP_CAN_Result BSP_CAN_ReadAvailable(
    BSP_CAN_Handle can, BSP_CAN_Frame *frame)
{
    can_rx_queue_t *queue;

    if (!can_handle_valid(can) || (frame == NULL)) {
        return BSP_CAN_RESULT_INVALID_ARGUMENT;
    }
    queue = &s_can1_rx_queue;
    if (queue->tail == queue->head) {
        return BSP_CAN_RESULT_NO_DATA;
    }
    *frame = queue->frames[queue->tail];
    queue->tail = (uint16_t)((queue->tail + 1U) % BSP_CAN_RX_QUEUE_SIZE);
    return BSP_CAN_RESULT_OK;
}

bool BSP_CAN_IsBusOff(BSP_CAN_Handle can)
{
    return can_handle_valid(can) && ((CAN1->ESR & CAN_ESR_BOFF) != 0U);
}

BSP_CAN_Result BSP_CAN_Recover(BSP_CAN_Handle can)
{
    if (!can_handle_valid(can)) {
        return BSP_CAN_RESULT_INVALID_ARGUMENT;
    }
    /* ABOM 未开：请求退出初始化模式触发总线关闭恢复序列。 */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    return BSP_CAN_RESULT_OK;
}

/* ------------------------------------------------------------------ */
/* BSP_Init                                                             */
/* ------------------------------------------------------------------ */

bool BSP_Init(void)
{
    if (!clock_init()) {
        return false;
    }

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
        RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN | RCC_APB1ENR_CAN1EN;
    (void)RCC->AHB1ENR;
    (void)RCC->APB1ENR;

    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
    RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;

    key_init();

    /* USART1: PA9 TX / PA10 RX AF7 */
    gpio_af_init(GPIOA, 9U, 7U);
    gpio_af_init(GPIOA, 10U, 7U);
    /* USART3: PB10 TX / PB11 RX AF7 */
    gpio_af_init(GPIOB, 10U, 7U);
    gpio_af_init(GPIOB, 11U, 7U);
    ring_reset(&s_uart1_tx);
    ring_reset(&s_uart1_rx);
    ring_reset(&s_uart3_tx);
    ring_reset(&s_uart3_rx);
    memset(&s_uart1_dma, 0, sizeof(s_uart1_dma));
    memset((void *)s_uart1_err, 0, sizeof(s_uart1_err));
    memset((void *)s_uart3_err, 0, sizeof(s_uart3_err));

    DMA2_Stream2->CR = 0U;
    DMA2_Stream7->CR = 0U;
    uart_periph_init(USART1, BSP_APB2_CLOCK_HZ, true);
    uart_periph_init(USART3, BSP_APB1_CLOCK_HZ, false);
    if (!uart_dma_start_rx(&s_uart_resources[0])) {
        return false;
    }

    NVIC_SetPriority(USART1_IRQn, BSP_UART_IRQ_PRIORITY);
    NVIC_SetPriority(USART3_IRQn, BSP_UART_IRQ_PRIORITY);
    NVIC_SetPriority(DMA2_Stream2_IRQn, BSP_DMA_IRQ_PRIORITY);
    NVIC_SetPriority(DMA2_Stream7_IRQn, BSP_DMA_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(USART1_IRQn);
    NVIC_ClearPendingIRQ(USART3_IRQn);
    NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);
    NVIC_ClearPendingIRQ(DMA2_Stream7_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART3_IRQn);
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);

    return can_init();
}
