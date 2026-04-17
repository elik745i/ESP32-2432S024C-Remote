#include <Arduino.h>
#include <esp_heap_caps.h>

extern "C" void *lvgl_malloc_psram(size_t size)
{
    if (size == 0) return nullptr;
#if defined(BOARD_ESP32S3_3248S035_N16R8) || defined(BOARD_UEDX32480035E_WB_A) || defined(BOARD_JC4880P443C_I_W)
    if (psramFound()) {
        void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (ptr) return ptr;
    }
#endif
    return heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

extern "C" void lvgl_free_psram(void *ptr)
{
    if (!ptr) return;
    heap_caps_free(ptr);
}

extern "C" void *lvgl_realloc_psram(void *ptr, size_t size)
{
    if (!ptr) return lvgl_malloc_psram(size);
    if (size == 0) {
        lvgl_free_psram(ptr);
        return nullptr;
    }

#if defined(BOARD_ESP32S3_3248S035_N16R8) || defined(BOARD_UEDX32480035E_WB_A) || defined(BOARD_JC4880P443C_I_W)
    if (psramFound()) {
        void *moved = heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (moved) return moved;
    }
#endif
    return heap_caps_realloc(ptr, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}
