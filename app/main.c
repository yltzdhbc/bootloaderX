#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"

#include "bsp_can.h"
#include "bsp_fmc.h"
#include "app_protocol.h"

uint32_t current_time = 0;
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
        current_time = sys_tick_ms_get();

        if (current_time - time_last[0] > 2)
        {
            time_last[0] = current_time;
            app_protocol_loop();
        }
    }
}
