/*
 * Copyright (C) 2022 DJI.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-27     robomaster   first version
 */

#include "open_protocol_cmd.h"
#include "open_protocol.h"
// #include "ext_protocol.h"
#include "string.h"
#include "crc.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "sys_param.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint16_t cmd_id;
    open_cmd_handler_t handler;
} open_handler_keypair_t;

// Open Protocol 协议回调表
const open_handler_keypair_t open_protocol_fun_req[] =
    {
        {OPEN_CMD_REBOOT, open_cmd_reboot},
        {OPEN_CMD_VERSION, open_cmd_ver},
        {OPEN_CMD_ECHO, open_cmd_echo},
        {OPEN_CMD_SET_ID, open_cmd_set_id},
        {OPEN_CMD_UPGRADE_ENTER, open_cmd_enter_loader},

        {OPEN_CMD_UPGRADE_INFO, upgrade_info_pack_handle},
        {OPEN_CMD_UPGRADE_DATA, upgrade_data_pack_handle},
        {OPEN_CMD_UPGRADE_END, upgrade_end_pack_handle},
        {OPEN_CMD_STOP_BOOT_APP, open_cmd_stop_boot_app},

        // {OPEN_CMD_LED_TEST, open_cmd_led_test},
        // {OPEN_CMD_SD_CARD_TEST, open_cmd_sd_card_test},
        // {OPEN_CMD_CAN_TEST, open_cmd_can_test},
        // {OPEN_CMD_SLOT14_TEST, open_cmd_slot14_test},
        // {OPEN_CMD_SLOT100_TEST, open_cmd_slot100_test},
        // {OPEN_CMD_EN_AI_BOARD_IMG, open_cmd_en_ai_board_img},
        // {OPEN_CMD_BURN_SN, open_cmd_burn_sn},

        // {OPEN_CMD_V1_VER, open_cmd_v1_ver},

        // {OPEN_CMD_OLD_PROTO_REQ_ACK_FOWARD, open_cmd_proxy_common}
};

static uint8_t can_port_idx;
static uint8_t uart0_port_idx;

void app_protocol_init(void)
{
    open_proto_init(g_sys_param.device_id | 0x0200);

    can_port_idx = open_proto_port_add("CAN", can_send, can_receive);
    open_proto_static_route_add(0x0, 0x0, can_port_idx, 253);

    uart0_port_idx = open_proto_port_add("UART0", uart0_send, uart0_receive);
    open_proto_static_route_add(0x03FF, 0xFF00, uart0_port_idx, 2);

    for (int i = 0; i < sizeof(open_protocol_fun_req) / sizeof(open_handler_keypair_t); i++)
    {
        open_proto_handle_reg(open_protocol_fun_req[i].cmd_id, open_protocol_fun_req[i].handler);
    }
}

void app_protocol_loop(void)
{
    open_proto_recv();
}