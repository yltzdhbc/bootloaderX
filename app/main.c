#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"

#include "bsp_uart.h"
#include "bsp_can.h"
#include "bsp_fmc.h"
#include "app_protocol.h"

uint8_t g_app_start = 1;

uint32_t time_now = 0;
uint32_t time_last[3] = {0};

/*!
    \brief    main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    systick_config();

    bsp_can_init();
    bsp_fmc_init();
    bsp_uart_init();

    app_protocol_init();

    while (1)
    {
        time_now = sys_tick_ms_get();

        if (time_now - time_last[0] > 2)
        {
            time_last[0] = time_now;
            app_protocol_loop();
        }
    }
}

// /* retarget the C library printf function to the USART */
// int fputc(int ch, FILE *f)
// {
//     usart_data_transmit(USART0, (uint8_t)ch);
//     // while (RESET == usart_flag_get(USART0, USART_FLAG_TBE))
//     //     ;
//     return ch;
// }
