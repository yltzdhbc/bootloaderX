/*
 * Copyright (C) 2022 DJI.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-27     robomaster   first version
 */

#ifndef __EXT_PROTOCOL_COMMON_H__
#define __EXT_PROTOCOL_COMMON_H__

#include "stdint.h"
#include "ext_protocol.h"


#pragma pack(push, 1)

/* ??竹 */
#define EXT_CMD_REBOOT_REQ        (0x0002)

#define EXT_CMD_REBOOT_RSP        (0x0003)
typedef struct
{
    uint8_t error;
}ext_cmd_reboot_rsp_t;


/* ????煆 */
#define EXT_CMD_VERSION_REQ        (0x0004)

#define EXT_CMD_VERSION_RSP        (0x0005)
typedef struct
{
    uint32_t type;
    uint32_t loader_ver;
    uint32_t app_ver;
    char     sn[16];
}ext_cmd_version_rsp_t;


/* SPI??????? */
#define EXT_SPI_VEDIO_STREAM_REQ        (0x0008)
typedef struct
{
    uint8_t open;
}ext_spi_vedio_req_t;

#define EXT_SPI_VEDIO_STREAM_RSP        (0x0009)
typedef struct
{
    uint8_t error;
}ext_spi_vedio_rsp_t;

/* AI???????? */
#define EXT_CMD_DETECT_TAG_PUSH         (0x0101)
typedef struct
{
    uint8_t  id;    /*!< ???ID?  */
    uint16_t x;     /*!< ?????????快?x??????  */
    uint8_t  y;     /*!< ?????????快?y??????  */
    uint16_t w;     /*!< ?????????快????  */
    uint8_t  h;     /*!< ?????????快???  */
    uint8_t  C;     /*!< ???????? 0~100 */
}ext_comm_detect_tag_t;


/* ??????????? Step1: ????Loader */
#define EXT_CMD_ENTER_BOOTLOADER_REQ        (0x0402)
typedef struct
{
    uint8_t encrypt;
}ext_cmd_enter_bootloader_req_t;

#define EXT_CMD_ENTER_BOOTLOADER_RSP        (0x0403)
typedef struct
{
    uint8_t error;
}ext_cmd_enter_bootloader_rsp_t;

/* ??????????? Step2: ???????????????*/
#define EXT_CMD_UPGRADE_FIRMWARE_INFO_REQ   (0x0404)
typedef struct
{
    uint8_t encrypt;
    uint32_t size;
}ext_cmd_upgrade_firmware_info_req_t;

#define EXT_CMD_UPGRADE_FIRMWARE_INFO_RSP   (0x0405)
typedef struct
{
    uint8_t error;
}ext_cmd_upgrade_firmware_info_rsp_t;


/* ??????????? Step3: ??????????*/
#define EXT_CMD_UPGRADE_FIRMWARE_DATA_REQ   (0x0406)
typedef struct
{
    uint8_t encrypt;
    uint32_t pack_idx;
    uint32_t pack_size;
    uint8_t  m_data[256];
}ext_cmd_upgrade_firmware_data_req_t;

#define EXT_CMD_UPGRADE_FIRMWARE_DATA_RSP   (0x0407)
typedef struct
{
    uint8_t error;
}ext_cmd_upgrade_firmware_data_rsp_t;


/* ??????????? Step4: ??????????*/
#define EXT_CMD_UPGRADE_FIRMWARE_END_REQ    (0x0408)
typedef struct
{
    uint8_t encrypt;
    uint8_t md5[16];
}ext_cmd_upgrade_firmware_end_req_t;

#define EXT_CMD_UPGRADE_FIRMWARE_END_RSP    (0x0409)
typedef struct
{
    uint8_t error;
}ext_cmd_upgrade_firmware_end_rsp_t;

#pragma pack(pop)

void ext_cmd_default(ext_protocol_header_t *pack_desc);
void ext_cmd_all(ext_protocol_header_t *pack_desc);
void ext_cmd_version_rsp(ext_protocol_header_t *pack_desc);

void ext_cmd_commmon_forward_handle(ext_protocol_header_t *pack_desc);

#endif
