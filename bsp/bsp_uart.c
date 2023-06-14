#include "bsp_uart.h"
// #include "ringbuffer.h"
#include "ringbuffer.h"

#include <stdio.h>

#define COMn 1U
#define EVAL_COM0 USART0
#define EVAL_COM0_CLK RCU_USART0

#define EVAL_COM0_TX_PIN GPIO_PIN_9
#define EVAL_COM0_RX_PIN GPIO_PIN_10

#define EVAL_COM0_GPIO_PORT GPIOA
#define EVAL_COM0_GPIO_CLK RCU_GPIOA
#define EVAL_COM0_AF GPIO_AF_7

static rcu_periph_enum COM_CLK[COMn] = {EVAL_COM0_CLK};
static uint32_t COM_TX_PIN[COMn] = {EVAL_COM0_TX_PIN};
static uint32_t COM_RX_PIN[COMn] = {EVAL_COM0_RX_PIN};

// #define TX_FIFO_SIZE 512
#define RX_FIFO_SIZE 256
// volatile static char tx_fifo_buff[TX_FIFO_SIZE] = {0};
// volatile static char rx_fifo_buff[RX_FIFO_SIZE] = {0};
// fifo_t tx_fifo;
// fifo_t rx_fifo;

// static RingBuffer *rx_cbuffer = NULL;

// #define S_RING_BUFF_ELEM_TYPE char
// #define S_RING_BUFF_ELEM_CAP RX_FIFO_SIZE
// s_ring_buffer_t *s_ring_buffer_g_p = NULL;
// #define READ_WRITE_BUFF_CAP S_RING_BUFF_ELEM_CAP * 6

// char rx_cbuffer_buff[RX_FIFO_SIZE] = {0};

ring_buffer_t ring_buffer;
char buf_arr[RX_FIFO_SIZE];

void USART0_IRQHandler(void)
{
    if ((RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)) &&
        (RESET != usart_flag_get(USART0, USART_FLAG_RBNE)))
    {
        usart_flag_clear(USART0, USART_FLAG_RBNE);
        char chr = usart_data_receive(USART0);
        // fifo_puts(&rx_fifo, (char *)&chr, 1);
        // RingBuffer_write(rx_cbuffer, &chr, sizeof(chr));

        // s_ring_buffer_write_elements(s_ring_buffer_g_p, (void *)&chr, READ_WRITE_BUFF_CAP, 1);

        ring_buffer_queue(&ring_buffer, chr);
    }
}

/**
 * @brief 串口0发送函数
 *
 * @param data
 * @param len
 */
void uart0_send(uint8_t *data, uint16_t len)
{
    for (int i = 0; i < len; i++)
    {
        usart_data_transmit(EVAL_COM0, (uint32_t)data[i]);
        while(RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TBE));
    }
}

/**
 * @brief 串口0接收
 *
 * @param data
 * @param len
 */
uint16_t uart0_receive(uint8_t *buff, uint16_t len)
{
    // NVIC_DisableIRQ();

    // __disable_irq();

    // fifo_gets(&rx_fifo, (char *)&buff, len);

    // uint16_t readlen = RingBuffer_read(rx_cbuffer, buff, len);

    // uint16_t readlen = s_ring_buffer_read_elements(s_ring_buffer_g_p, buff, len, 1);

    uint16_t readlen = ring_buffer_dequeue_arr(&ring_buffer, buff, len);

    // NVIC_EnableIRQ();

    // __enable_irq();

    return readlen;

    // return fifo_gets(&rx_fifo, (char *)&buff, len);
}

void uart_com_init(uint32_t com)
{
    // 初始化接收缓冲
    // fifo_init(&tx_fifo, tx_fifo_buff, TX_FIFO_SIZE);
    // fifo_init(&rx_fifo, rx_fifo_buff, RX_FIFO_SIZE);
    // rx_cbuffer = RingBuffer_create(RX_FIFO_SIZE);

    // s_ring_buffer_g_p = s_ring_buffer_constructor(sizeof(S_RING_BUFF_ELEM_TYPE), S_RING_BUFF_ELEM_CAP, NULL, NULL);

    ring_buffer_init(&ring_buffer, buf_arr, sizeof(buf_arr));

    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* configure the USART0 TX pin and USART0 RX pin */
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_9);
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_10);
    /* configure USART0 TX as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    /* configure USART0 RX as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

    /* USART interrupt configuration */
    nvic_irq_enable(USART0_IRQn, 0, 0);
    // usart_interrupt_enable(USART0, USART_INT_TBE);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
}

void bsp_uart_init(void)
{
    uart_com_init(USART0);
}
