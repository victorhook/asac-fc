#include "ringbuf.h"
#include <string.h>


void ringbuf_init(ringbuf_t* buf, uint8_t* data, const uint32_t size)
{
    buf->data = data;
    buf->read_index = 0;
    buf->write_index = 0;
    buf->size = size;
    buf->items = 0;
}

bool ringbuf_get(ringbuf_t* buf, uint8_t* byte)
{
    if (buf->items == 0) return false;

    *byte = buf->data[buf->read_index];
    buf->read_index = (buf->read_index + 1) % buf->size;
    buf->items--;

    return true;
}

bool ringbuf_add(ringbuf_t* buf, const uint8_t byte)
{
    if (buf->items == buf->size) return false;
    
    buf->data[buf->write_index] = byte;
    buf->write_index = (buf->write_index + 1) % buf->size;
    buf->items++;

    return true;
}

bool ringbuf_add_bytes(ringbuf_t* buf, const uint8_t* data, const uint16_t len)
{
    if (len > ringbuf_free_space(buf)) return false;

    // How many bytes can we write before wrapping around
    uint16_t first_chunk = buf->size - buf->write_index;
    if (first_chunk > len) {
        first_chunk = len;
    }

    // Copy first chunk
    memcpy(&buf->data[buf->write_index], data, first_chunk);

    // Copy second chunk if wraparound is needed
    uint16_t second_chunk = len - first_chunk;
    if (second_chunk > 0) {
        memcpy(buf->data, data + first_chunk, second_chunk);
    }

    // Update state
    buf->write_index = (buf->write_index + len) % buf->size;
    buf->items += len;

    return true;
}


uint32_t ringbuf_peek(const ringbuf_t* buf, uint8_t** ptr)
{
    if (ringbuf_is_empty(buf))
    {
        *ptr = 0;
        return 0;
    }

    // Assign address of next data chunk
    *ptr = &buf->data[buf->read_index];

    if (buf->read_index < buf->write_index) {
        // Contiguous chunk before wrapping
        return buf->write_index - buf->read_index;
    } else {
        // Wraps around, so chunk goes until end of buffer
        return buf->size - buf->read_index;
    }
}

void ringbuf_advance(ringbuf_t* buf, const uint32_t nbr_of_bytes)
{
    buf->read_index = (buf->read_index + nbr_of_bytes) % buf->size;
    buf->items -= (nbr_of_bytes > buf->items) ? buf->items : nbr_of_bytes;
}
