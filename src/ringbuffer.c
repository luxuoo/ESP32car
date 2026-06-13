/* 镜像位环形缓冲区 — 移植自 STM32 方案, 无修改 */

#include "ringbuffer.h"

#define RB_ALIGN_SIZE 4
#define RB_ALIGN_DOWN(size, align) ((size) & ~((align) - 1))

enum rbuffer_state rbuffer_status(rbuffer_t *rb)
{
    if (rb->read_index == rb->write_index) {
        if (rb->read_mirror == rb->write_mirror)
            return RB_EMPTY;
        else
            return RB_FULL;
    }
    return RB_HALFFULL;
}

void rbuffer_init(rbuffer_t *rb, uint8_t *pool, int32_t size)
{
    rb->read_mirror = rb->read_index = 0;
    rb->write_mirror = rb->write_index = 0;
    rb->buffer_ptr  = pool;
    rb->buffer_size = RB_ALIGN_DOWN(size, RB_ALIGN_SIZE);
}

uint32_t rbuffer_put(rbuffer_t *rb, const uint8_t *ptr, uint32_t length)
{
    uint32_t size = rbuffer_space_len(rb);
    if (size == 0) return 0;
    if (size < length) length = size;

    if (rb->buffer_size - rb->write_index > length) {
        memcpy(&rb->buffer_ptr[rb->write_index], ptr, length);
        rb->write_index += length;
        return length;
    }

    memcpy(&rb->buffer_ptr[rb->write_index], &ptr[0], rb->buffer_size - rb->write_index);
    memcpy(&rb->buffer_ptr[0], &ptr[rb->buffer_size - rb->write_index],
           length - (rb->buffer_size - rb->write_index));

    rb->write_mirror = ~rb->write_mirror;
    rb->write_index  = length - (rb->buffer_size - rb->write_index);
    return length;
}

uint32_t rbuffer_put_force(rbuffer_t *rb, const uint8_t *ptr, uint32_t length)
{
    uint32_t space_length = rbuffer_space_len(rb);

    if (length > rb->buffer_size) {
        ptr = &ptr[length - rb->buffer_size];
        length = rb->buffer_size;
    }

    if (rb->buffer_size - rb->write_index > length) {
        memcpy(&rb->buffer_ptr[rb->write_index], ptr, length);
        rb->write_index += length;
        if (length > space_length)
            rb->read_index = rb->write_index;
        return length;
    }

    memcpy(&rb->buffer_ptr[rb->write_index], &ptr[0], rb->buffer_size - rb->write_index);
    memcpy(&rb->buffer_ptr[0], &ptr[rb->buffer_size - rb->write_index],
           length - (rb->buffer_size - rb->write_index));

    rb->write_mirror = ~rb->write_mirror;
    rb->write_index  = length - (rb->buffer_size - rb->write_index);

    if (length > space_length) {
        if (rb->write_index <= rb->read_index)
            rb->read_mirror = ~rb->read_mirror;
        rb->read_index = rb->write_index;
    }
    return length;
}

uint32_t rbuffer_get(rbuffer_t *rb, uint8_t *ptr, uint32_t length)
{
    uint32_t size = rbuffer_data_len(rb);
    if (size == 0) return 0;
    if (size < length) length = size;

    if (rb->buffer_size - rb->read_index > length) {
        memcpy(ptr, &rb->buffer_ptr[rb->read_index], length);
        rb->read_index += length;
        return length;
    }

    memcpy(&ptr[0], &rb->buffer_ptr[rb->read_index], rb->buffer_size - rb->read_index);
    memcpy(&ptr[rb->buffer_size - rb->read_index], &rb->buffer_ptr[0],
           length - (rb->buffer_size - rb->read_index));

    rb->read_mirror = ~rb->read_mirror;
    rb->read_index  = length - (rb->buffer_size - rb->read_index);
    return length;
}

uint32_t rbuffer_del(rbuffer_t *rb, uint32_t length)
{
    uint32_t size = rbuffer_data_len(rb);
    if (size == 0) return 0;
    if (size < length) length = size;

    if (rb->buffer_size - rb->read_index > length) {
        rb->read_index += length;
        return length;
    }

    rb->read_mirror = ~rb->read_mirror;
    rb->read_index  = length - (rb->buffer_size - rb->read_index);
    return length;
}

uint32_t rbuffer_peek(rbuffer_t *rb, uint8_t *ptr, uint32_t length)
{
    uint32_t size = rbuffer_data_len(rb);
    if (size == 0) return 0;
    if (size < length) length = size;

    if (rb->buffer_size - rb->read_index > length) {
        memcpy(ptr, &rb->buffer_ptr[rb->read_index], length);
        return length;
    }

    memcpy(&ptr[0], &rb->buffer_ptr[rb->read_index], rb->buffer_size - rb->read_index);
    memcpy(&ptr[rb->buffer_size - rb->read_index], &rb->buffer_ptr[0],
           length - (rb->buffer_size - rb->read_index));
    return length;
}

uint32_t rbuffer_putchar(rbuffer_t *rb, const uint8_t ch)
{
    if (!rbuffer_space_len(rb)) return 0;

    rb->buffer_ptr[rb->write_index] = ch;

    if (rb->write_index == rb->buffer_size - 1) {
        rb->write_mirror = ~rb->write_mirror;
        rb->write_index = 0;
    } else {
        rb->write_index++;
    }
    return 1;
}

uint32_t rbuffer_putchar_force(rbuffer_t *rb, const uint8_t ch)
{
    enum rbuffer_state old_state = rbuffer_status(rb);

    rb->buffer_ptr[rb->write_index] = ch;

    if (rb->write_index == rb->buffer_size - 1) {
        rb->write_mirror = ~rb->write_mirror;
        rb->write_index  = 0;
        if (old_state == RB_FULL) {
            rb->read_mirror = ~rb->read_mirror;
            rb->read_index = rb->write_index;
        }
    } else {
        rb->write_index++;
        if (old_state == RB_FULL)
            rb->read_index = rb->write_index;
    }
    return 1;
}

uint32_t rbuffer_getchar(rbuffer_t *rb, uint8_t *ch)
{
    if (!rbuffer_data_len(rb)) return 0;

    *ch = rb->buffer_ptr[rb->read_index];

    if (rb->read_index == rb->buffer_size - 1) {
        rb->read_mirror = ~rb->read_mirror;
        rb->read_index  = 0;
    } else {
        rb->read_index++;
    }
    return 1;
}

uint32_t rbuffer_data_len(rbuffer_t *rb)
{
    switch (rbuffer_status(rb)) {
    case RB_EMPTY:
        return 0;
    case RB_FULL:
        return rb->buffer_size;
    case RB_HALFFULL:
    default: {
        uint32_t wi = rb->write_index, ri = rb->read_index;
        if (wi > ri)
            return wi - ri;
        else
            return rb->buffer_size - (ri - wi);
    }
    }
}

void rbuffer_reset(rbuffer_t *rb)
{
    rb->read_mirror  = 0;
    rb->read_index   = 0;
    rb->write_mirror = 0;
    rb->write_index  = 0;
}
