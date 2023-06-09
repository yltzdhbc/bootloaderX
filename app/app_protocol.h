/*
 * Copyright (C) 2022 DJI.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-27     robomaster   first version
 */

#ifndef __TASK_PROCOTOL_H__
#define __TASK_PROCOTOL_H__

#include "stdint.h"

#define OPEN_PROTO_LOCAL_ADDR_BASE 0x0100u // 在开放协议中的地址起始

// void task_protocol(void const * argument);
void app_protocol_init(void);
void app_protocol_loop(void);

#endif //__TASK_DJI_PROCOTOL_H__
