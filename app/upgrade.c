#include "upgrade.h"
#include "bsp_fmc.h"

static fw_upgrade_status_e upgrade_status = UPGRADE_WAIT_FW_INFO;
static uint32_t fw_size = 0;
static uint32_t fw_pack_idx = 0;
static uint32_t flash_write_ptr = 0;
static uint16_t local_sn_crc16 = 0;
static char local_hw_id[16] = {0};
static uint8_t last_error = 0;
static uint8_t upgrade_is_end = 0;

static void upgrade_comm_ack(open_protocol_header_t *pack_desc, uint8_t error_code);

void upgrade_init(uint16_t sn_crc16, char *hw_id)
{
    local_sn_crc16 = sn_crc16;
    strcpy(local_hw_id, hw_id);
}

void upgrade_info_pack_handle(open_protocol_header_t *pack_desc)
{
    open_cmd_upgrade_info_req *req = (open_cmd_upgrade_info_req *)(pack_desc->data);

//    /* 匹配硬件码和SN的CRC16 */
//    if ((strncmp(local_hw_id, (char *)req->hw_id, 16) != 0) && (req->sn_crc16 != local_sn_crc16))
//    {
//        return;
//    }

    if (pack_desc->is_ack == 0)
    {
        uint32_t erase_bytes = UPGRADE_END_FLASH_ADDRESS - UPGRADE_START_FLASH_ADDRESS;
        if (pack_desc->data_len == sizeof(open_cmd_upgrade_info_req) && req->erase_bytes != 0)
        {
            erase_bytes = req->erase_bytes;
        }

        /* 判断固件大小和擦除大小是否在Flash范围内 */
        if ((req->fw_size > UPGRADE_END_FLASH_ADDRESS - UPGRADE_START_FLASH_ADDRESS) ||
            (req->erase_bytes > UPGRADE_END_FLASH_ADDRESS - UPGRADE_START_FLASH_ADDRESS))
        {
            upgrade_comm_ack(pack_desc, OPEN_PROTO_OVERSIZE);
            return;
        }

        /* 仅支持非加密类型 */
        if (req->encrypt != 0)
        {
            upgrade_comm_ack(pack_desc, OPEN_PROTO_INVALID_PARAM);
            return;
        }

        /*重新设置看门狗的时间 -> 25s*/

        /* 擦除Flash并将相关参数初始化 */
        if (FLASH_OK != flash_erase(UPGRADE_START_FLASH_ADDRESS, erase_bytes))
        {
            upgrade_comm_ack(pack_desc, OPEN_PROTO_FLASH_ERROR);
            /*重新设置看门狗的时间 -> 1.5s*/
            return;
        }

        fw_pack_idx = 0;
        fw_size = req->fw_size;
        flash_write_ptr = 0;
        upgrade_is_end = 0;
        upgrade_status = UPGRADE_TRANS_FW_DATA;

        upgrade_comm_ack(pack_desc, OPEN_PROTO_NORMAL);
    }
}

void upgrade_data_pack_handle(open_protocol_header_t *pack_desc)
{
    open_cmd_upgrade_data_req *req = (open_cmd_upgrade_data_req *)(pack_desc->data);
    uint32_t flash_write_num = MAX_SUPPORT_FW_PACK_SIZE;

    if (pack_desc->is_ack == 0 && req->sn_crc16 == local_sn_crc16)
    {
        /* 检查包序号和升级状态是否正确 */
        if (fw_pack_idx < req->pack_idx || upgrade_status != UPGRADE_TRANS_FW_DATA)
        {
            upgrade_comm_ack(pack_desc, OPEN_PROTO_IDX_ERROR);
            return;
        }

        /* 除了最后一个包，其他包长必须为MAX_SUPPORT_FW_PACK_SIZE */
        if (fw_size - flash_write_ptr > MAX_SUPPORT_FW_PACK_SIZE && req->pack_size != MAX_SUPPORT_FW_PACK_SIZE)
        {
            upgrade_comm_ack(pack_desc, OPEN_PROTO_WRONG_LENGTH);
            return;
        }

        /* 写入升级数据 */
        if (req->pack_idx == fw_pack_idx)
        {
            if (FLASH_OK != flash_write(UPGRADE_START_FLASH_ADDRESS + flash_write_ptr, req->fw_data, req->pack_size))
            {
                upgrade_comm_ack(pack_desc, OPEN_PROTO_FLASH_ERROR);
                return;
            }
        }

        upgrade_comm_ack(pack_desc, OPEN_PROTO_NORMAL);
        return;
    }
}

void upgrade_end_pack_handle(open_protocol_header_t *pack_desc)
{
    open_cmd_upgrade_end_req *req = (open_cmd_upgrade_end_req *)(pack_desc->data);
    uint32_t flash_write_num = MAX_SUPPORT_FW_PACK_SIZE;
    uint8_t fw_md5[16];

    if (pack_desc->is_ack == 0 && req->sn_crc16 == local_sn_crc16)
    {
        /* 检查包序号和升级状态是否正确 */
        if (upgrade_status != UPGRADE_WAIT_END)
        {
            upgrade_comm_ack(pack_desc, OPEN_PROTO_IDX_ERROR);
            return;
        }

        // 計算MO5进行对比

        // 写入系统参数

        upgrade_is_end = 1;

        upgrade_comm_ack(pack_desc, OPEN_PROTO_NORMAL);
        return;
    }
}

int upgrade_check_app(uint8_t *app_md5, uint32_t app_size)
{
}

static void upgrade_comm_ack(open_protocol_header_t *pack_desc, uint8_t error_code)
{
    open_cmd_upgrade_comm_rsp_t rsp;
    rsp.err_code = error_code;
    last_error = error_code;

    if (pack_desc->need_ack)
    {
        open_proto_ack(pack_desc, (uint8_t *)(&rsp), sizeof(rsp));
    }
    return;
}
