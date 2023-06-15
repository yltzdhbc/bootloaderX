#ifndef __UPGRADE_H__
#define __UPGRADE_H__

#include "gd32f4xx.h"
#include "open_protocol_cmd.h"
#include "open_protocol_error.h"
#include <string.h>

#define MAX_SUPPORT_FW_PACK_SIZE (256)
#define UPGRADE_START_FLASH_ADDRESS (0x08020000)
#define UPGRADE_END_FLASH_ADDRESS   (0x0809FFFF)

typedef enum
{
    UPGRADE_WAIT_FW_INFO,
    UPGRADE_TRANS_FW_DATA,
    UPGRADE_WAIT_END,
} fw_upgrade_status_e;

#pragma pack(push, 1)

typedef struct
{
    uint8_t err_code;
} open_cmd_upgrade_comm_rsp_t;

#define OPEN_CMD_UPGRADE_ENTER (0x0021)

typedef struct
{
    uint8_t encrypt;
    uint16_t sn_crc16;
} open_cmd_upgrade_enter_req;

#define OPEN_CMD_UPGRADE_INFO (0x0022)

typedef struct
{
    uint8_t encrypt;
    uint32_t fw_size;
    uint16_t sn_crc16;
    uint8_t hw_id[16];
    uint32_t erase_bytes;
} open_cmd_upgrade_info_req;

#define OPEN_CMD_UPGRADE_DATA (0x0023)

typedef struct
{
    uint8_t encrypt;
    uint32_t pack_idx;
    uint16_t pack_size;
    uint16_t sn_crc16;
    uint8_t fw_data[256];
} open_cmd_upgrade_data_req;

#define OPEN_CMD_UPGRADE_END (0x0024)

typedef struct
{
    uint8_t encrypt;
    uint8_t md5[16];
    uint16_t sn_crc16;
} open_cmd_upgrade_end_req;

#pragma pack(pop)

void upgrade_init(uint16_t sn_crc16, char *hw_id);
void upgrade_info_pack_handle(open_protocol_header_t *pack_desc);
void upgrade_data_pack_handle(open_protocol_header_t *pack_desc);
void upgrade_end_pack_handle(open_protocol_header_t *pack_desc);
int upgrade_check_app(uint8_t *app_md5, uint32_t app_size);

#endif /* __UPGRADE_H__ */
