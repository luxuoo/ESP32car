#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#include <stdint.h>
#include <assert.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 镜像位环形缓冲区 (移植自 STM32 方案) */
struct rbuffer {
    uint8_t *buffer_ptr;
    uint32_t read_mirror : 1;
    uint32_t read_index  : 31;
    uint32_t write_mirror : 1;
    uint32_t write_index  : 31;
    int32_t  buffer_size;
};

typedef struct rbuffer rbuffer_t;

enum rbuffer_state {
    RB_EMPTY,
    RB_FULL,
    RB_HALFFULL,
};

#define rbuffer_space_len(rb) ((rb)->buffer_size - rbuffer_data_len(rb))

void     rbuffer_init(rbuffer_t *rb, uint8_t *pool, int32_t size);
void     rbuffer_reset(rbuffer_t *rb);
uint32_t rbuffer_put(rbuffer_t *rb, const uint8_t *ptr, uint32_t length);
uint32_t rbuffer_put_force(rbuffer_t *rb, const uint8_t *ptr, uint32_t length);
uint32_t rbuffer_putchar(rbuffer_t *rb, const uint8_t ch);
uint32_t rbuffer_putchar_force(rbuffer_t *rb, const uint8_t ch);
uint32_t rbuffer_del(rbuffer_t *rb, uint32_t length);
uint32_t rbuffer_get(rbuffer_t *rb, uint8_t *ptr, uint32_t length);
uint32_t rbuffer_peek(rbuffer_t *rb, uint8_t *ptr, uint32_t length);
uint32_t rbuffer_getchar(rbuffer_t *rb, uint8_t *ch);
uint32_t rbuffer_data_len(rbuffer_t *rb);
enum rbuffer_state rbuffer_status(rbuffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif
