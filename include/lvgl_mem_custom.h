#ifndef LVGL_MEM_CUSTOM_H
#define LVGL_MEM_CUSTOM_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void *lvgl_malloc_psram(size_t size);
void lvgl_free_psram(void *ptr);
void *lvgl_realloc_psram(void *ptr, size_t size);

#ifdef __cplusplus
}
#endif

#endif
