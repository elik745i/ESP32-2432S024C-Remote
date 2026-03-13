#ifndef LV_CONF_H
#define LV_CONF_H

#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0

#define LV_MEM_CUSTOM 1
#define LV_MEM_CUSTOM_INCLUDE "lvgl_mem_custom.h"
#define LV_MEM_CUSTOM_ALLOC lvgl_malloc_psram
#define LV_MEM_CUSTOM_FREE lvgl_free_psram
#define LV_MEM_CUSTOM_REALLOC lvgl_realloc_psram
#define LV_MEM_SIZE (32U * 1024U)

#define LV_TICK_CUSTOM 0

#define LV_USE_LOG 0

#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_DEFAULT &lv_font_montserrat_16

#define LV_USE_PERF_MONITOR 0
#define LV_USE_MEM_MONITOR 0
#define LV_USE_SNAPSHOT 1

#endif
