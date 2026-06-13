/* 通信协议 — 移植自 STM32 方案, 无修改
 * 帧格式: [0xAA] [len_lo] [len_hi] [payload...] [checksum] [0x55]
 */

#include "comm_protocol.h"
#include <string.h>

#define HEAD 0xAA
#define TAIL 0x55

static uint8_t check_sum(uint8_t *data, uint32_t len)
{
    uint8_t s = 0;
    for (uint32_t i = 0; i < len; i++)
        s += data[i];
    return s;
}

int32_t packet_check_valid(uint8_t *data, uint32_t len, uint32_t *redundant)
{
    if (!data || !len || !redundant)
        return -1;

    uint32_t index = 0;
    for (index = 0; index < len; index++) {
        if (data[index] == HEAD)
            break;
    }
    *redundant = index;

    if ((len - index) < 3)
        return -2;

    uint16_t payload_len = 0;
    memcpy(&payload_len, &data[index + 1], 2);
    if ((len - index) < (payload_len + 5))
        return -2;

    if ((data[index + 3 + payload_len + 1] != TAIL) ||
        (check_sum(&data[index + 1], 2 + payload_len) != data[index + 3 + payload_len]))
    {
        return -3;
    }

    return 0;
}

uint32_t packet_length(uint8_t *data, uint32_t len)
{
    if (!data || data[0] != HEAD || len < 5)
        return 0;

    uint16_t payload_len = 0;
    memcpy(&payload_len, &data[1], 2);
    return (3 + payload_len + 2);
}

int32_t packet_encode(uint8_t *payload, uint32_t len, uint8_t *packet_buff, uint32_t buff_len)
{
    if (!payload || !packet_buff || ((len + 5) > buff_len))
        return -1;

    uint16_t payload_len = len;
    uint32_t idx = 0;

    packet_buff[idx++] = HEAD;
    memcpy(&packet_buff[idx], &payload_len, 2);
    idx += 2;
    memcpy(&packet_buff[idx], payload, len);
    idx += len;

    uint8_t chk = check_sum(&packet_buff[1], 2 + len);
    packet_buff[idx++] = chk;
    packet_buff[idx++] = TAIL;

    return idx;
}

int32_t packet_decode(uint8_t *data, uint32_t len, uint8_t *payload_buff, uint32_t buff_len)
{
    if (!data || data[0] != HEAD || len < 5 || !payload_buff)
        return -1;

    uint16_t payload_len = 0;
    memcpy(&payload_len, &data[1], 2);
    if ((len < (payload_len + 5)) || (buff_len < payload_len))
        return -1;

    memcpy(payload_buff, &data[3], payload_len);
    return payload_len;
}
