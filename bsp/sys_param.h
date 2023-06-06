#ifndef __SYS_PARAM_H__
#define __SYS_PARAM_H__

#include <stdint.h>

#pragma pack(push, 1)

typedef struct
{
    uint32_t device_id;
    uint32_t boot_times;
} sys_cfg_param_t;

#pragma pack(pop)

extern sys_cfg_param_t g_sys_param;

#endif /* __SYS_PARAM_H__ */