#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "gd32f4xx.h"

// #include "fifo.h"

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define USART1_DMA_TX_BUFFER_SIZE       (256)
#define USART1_DMA_RX_BUFFER_SIZE       (256)

#define USART3_DMA_TX_BUFFER_SIZE       (256)
#define USART3_DMA_RX_BUFFER_SIZE       (256)

typedef uint32_t (*uart_rx_handler_t)(char* data, uint32_t num);  /*!< 解包回调函数  */

// typedef struct
// {
//     uint32_t                usart_periph;
//     // char*               	dma_tx_buff;
//     // uint32_t            	dma_tx_buff_size;
//     // char*               	dma_rx_buff;
//     // uint32_t            	dma_rx_buff_size;
//     // lpuart_edma_handle_t*   edma_periph;

//     uart_rx_handler_t   rx_handler;
//     uint8_t				tcd_flag;

//     fifo_t   tx_fifo;
//     fifo_t   rx_fifo;
//     uint8_t  is_sending;
//     uint32_t recv_idle_cnt;
// }uart_dev_t;

void uart0_send(uint8_t *data, uint16_t len);
uint16_t uart0_receive(uint8_t *buff, uint16_t len);
void bsp_uart_init(void);
void bsp_uart_com_init(uint32_t com);

#endif /* __BSP_UART_H__ */
