#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__
#include "gd32f4xx.h"
#include <stdio.h>

void bsp_can_init(void);

uint16_t can_receive(uint8_t *buf, uint16_t buf_size);
void can_send(uint8_t *buf, uint16_t len);
#endif /* __BSP_CAN_H__ */


