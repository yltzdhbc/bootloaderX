#include "bsp_uart.h"

uart_dev_t uart_dev[] =
    {
        {USART0, usart1_dma_tx_buff, USART1_DMA_TX_BUFFER_SIZE, usart1_dma_rx_buff, USART1_DMA_RX_BUFFER_SIZE, &USART0_LPUART_eDMA_Handle},
        {USART1, USART1_dma_tx_buff, USART1_DMA_TX_BUFFER_SIZE, USART1_dma_rx_buff, USART1_DMA_RX_BUFFER_SIZE, &USART1_LPUART_eDMA_Handle},
};

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

void bsp_uart_com_init(uint32_t com)
{
    /* enable GPIO clock */
    uint32_t COM_ID = 0;
    if (EVAL_COM0 == com)
    {
        COM_ID = 0U;
    }

    rcu_periph_clock_enable(EVAL_COM0_GPIO_CLK);

    /* enable USART clock */
    rcu_periph_clock_enable(COM_CLK[COM_ID]);

    /* connect port to USARTx_Tx */
    gpio_af_set(EVAL_COM0_GPIO_PORT, EVAL_COM0_AF, COM_TX_PIN[COM_ID]);

    /* connect port to USARTx_Rx */
    gpio_af_set(EVAL_COM0_GPIO_PORT, EVAL_COM0_AF, COM_RX_PIN[COM_ID]);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(EVAL_COM0_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, COM_TX_PIN[COM_ID]);
    gpio_output_options_set(EVAL_COM0_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, COM_TX_PIN[COM_ID]);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(EVAL_COM0_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, COM_RX_PIN[COM_ID]);
    gpio_output_options_set(EVAL_COM0_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, COM_RX_PIN[COM_ID]);

    /* USART configure */
    usart_deinit(com);
    usart_baudrate_set(com, 115200U);
    usart_receive_config(com, USART_RECEIVE_ENABLE);
    usart_transmit_config(com, USART_TRANSMIT_ENABLE);
    usart_enable(com);
}

void USART0_IRQHandler(void)
{
    if ((RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)) &&
        (RESET != usart_flag_get(USART0, USART_FLAG_RBNE)))
    {
        usart_flag_clear(USART0, USART_FLAG_RBNE);
        /* receive data */
        unsigned char chr = usart_data_receive(USART0);

        uart_dev_t *obj = uart_get_obj(USART0);
        uart_receive_to_fifo(obj, &chr, 1);
    }
}

/**
 * @brief 获取UART设备结构体
 *
 * @param uart_periph
 * @return uart_dev_t*
 */
static uart_dev_t *uart_get_obj(uint32_t *uart_periph)
{
    switch ((uint32_t)uart_periph)
    {
    case (uint32_t)(USART0):
        return uart_dev + 0;
    case (uint32_t)(USART1):
        return uart_dev + 1;
    default:
        return uart_dev + 0;
    }
}

static void uart_receive_to_fifo(uart_dev_t *obj, char *data, uint16_t len)
{
    uint32_t ret = 0;
    if (obj->rx_handler != NULL)
    {
        ret = (obj->rx_handler)(data, len);
        ret = ret > len ? len : ret;
    }
    fifo_puts(&(obj->rx_fifo), data + ret, len - ret);
}

void uart0_init(uint32_t baud)
{
    /* USART interrupt configuration */
    nvic_irq_enable(USART0_IRQn, 0, 0);
    /* configure COM0 */
    gd_eval_com_init(EVAL_COM0);
    /* enable USART TBE interrupt */
    usart_interrupt_enable(USART0, USART_INT_TBE);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
}

/**
 * @brief UART设备初始化
 *
 * @param uart_periph
 * @param baudrate
 * @param tx_fifo_size
 * @param rx_fifo_size
 */
void uart_init(uint32_t *uart_periph, uint32_t baudrate, uint32_t tx_fifo_size, uint32_t rx_fifo_size)
{
    uart_dev_t *obj;
    switch ((uint32_t)uart_periph)
    {
    case (uint32_t)(USART0):
        uart0_init(baudrate);
        obj = uart_dev + 0;
        break;
    case (uint32_t)(USART1):
        // uart1_init(baudrate);
        obj = uart_dev + 1;
        break;
    default:
        return;
    }

    // char *tx_fifo_buff = pvPortMalloc(tx_fifo_size);
    // char *rx_fifo_buff = pvPortMalloc(rx_fifo_size);
    char *tx_fifo_buff = malloc(tx_fifo_size);
    char *rx_fifo_buff = malloc(rx_fifo_size);

    ASSERT(tx_fifo_buff != NULL);
    ASSERT(rx_fifo_buff != NULL);

    fifo_init(&(obj->tx_fifo), tx_fifo_buff, tx_fifo_size);
    fifo_init(&(obj->rx_fifo), rx_fifo_buff, rx_fifo_size);

    obj->tcd_flag = 0;
}

/**
 * @brief UART发送数据
 *
 * @param uart_periph
 * @param data
 * @param len
 */
void uart_send(uint32_t *uart_periph, char *data, uint16_t len)
{
    // unsigned long cpu_sr = __get_PRIMASK();
    // __disable_irq();
    // lpuart_transfer_t sendXfer;

    uart_dev_t *obj = uart_get_obj(uart_periph);

    for (int i = 0; i < len; i++)
    {
        usart_data_transmit(EVAL_COM0, (uint8_t)&data[i]);
    }

    // if (obj->is_sending)
    // {
    //     fifo_puts(&(obj->tx_fifo), data, len);
    // }
    // else
    // {
    //     memset(&sendXfer, 0, sizeof(sendXfer));

    //     if (len > obj->dma_tx_buff_size)
    //     {
    //         memcpy(obj->dma_tx_buff, data, obj->dma_tx_buff_size);

    //         sendXfer.data = obj->dma_tx_buff;
    //         sendXfer.dataSize = obj->dma_tx_buff_size;
    //         fifo_puts(&(obj->tx_fifo), data + obj->dma_tx_buff_size, len - obj->dma_tx_buff_size);
    //     }
    //     else
    //     {
    //         memcpy(obj->dma_tx_buff, data, len);
    //         sendXfer.data = obj->dma_tx_buff;
    //         sendXfer.dataSize = len;
    //     }
    //     obj->is_sending = 1;
    //     LPUART_SendEDMA(obj->usart_periph, obj->edma_periph, &sendXfer);
    // }

    // __set_PRIMASK(cpu_sr);
}

/**
 * @brief UART接收数据
 *
 * @param uart_periph
 * @param data
 * @param len
 * @return int
 */
int uart_receive(uint32_t *uart_periph, char *data, uint16_t len)
{

    uart_dev_t *obj = uart_get_obj(uart_periph);
    int32_t ret;

    // unsigned long cpu_sr = __get_PRIMASK();
    // __disable_irq();

    ret = fifo_gets(&(obj->rx_fifo), data, len);

    // __set_PRIMASK(cpu_sr);

    return ret;
}

/**
 * @brief 注册接收回调函数
 *
 * @param uart_periph
 * @param handler
 */
void uart_rx_handler_reg(uint32_t *uart_periph, uart_rx_handler_t handler)
{
    uart_dev_t *obj = uart_get_obj(uart_periph);
    obj->rx_handler = handler;
}

/**
 * @brief 串口0发送函数
 *
 * @param data
 * @param len
 */
void uart0_send(uint8_t *data, uint16_t len)
{
    uart_send(USART0, (char *)(data), len);
}

/**
 * @brief 串口0接收
 *
 * @param data
 * @param len
 */
uint16_t uart0_receive(uint8_t *buff, uint16_t len)
{
    return uart_receive(USART0, (char *)(buff), len);
}

void bsp_uart_init(void)
{
    uart_init(USART0, 115200, 256, 256);
}
