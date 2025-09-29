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

bool ringbuf_read(ringbuf_t* buf, uint8_t* data, const uint32_t len)
{
    if (len > buf->items) return false; // Not enough data in buffer

    uint32_t first_chunk = buf->size - buf->read_index;

    if (first_chunk > len)
    {   
        first_chunk = len; // Limit to max len
    }

    // Copy first chunk
    memcpy(data, &buf->data[buf->read_index], first_chunk);

    // Copy second chunk (if wrapped, so we're starting from beginning of buffer)
    uint32_t second_chunk = len - first_chunk;
    if (second_chunk > 0)
    {
        memcpy(data + first_chunk, buf->data, second_chunk);
    }

    // Advance read index
    buf->read_index = (buf->read_index + len) % buf->size;
    buf->items -= len;
    return true;
}

bool ringbuf_read_byte(ringbuf_t* buf, uint8_t* byte)
{
    if (buf->items == 0) return false;

    *byte = buf->data[buf->read_index];
    buf->read_index = (buf->read_index + 1) % buf->size;
    buf->items--;

    return true;
}

bool ringbuf_write_byte(ringbuf_t* buf, const uint8_t byte)
{
    if (buf->items == buf->size) return false;
    
    buf->data[buf->write_index] = byte;
    buf->write_index = (buf->write_index + 1) % buf->size;
    buf->items++;

    return true;
}

bool ringbuf_write(ringbuf_t* buf, const uint8_t* data, const uint32_t len)
{
    if (len > ringbuf_free_space(buf)) return false;

    // How many bytes can we write before wrapping around
    uint32_t first_chunk = buf->size - buf->write_index;
    if (first_chunk > len) {
        first_chunk = len;
    }

    // Copy first chunk
    memcpy(&buf->data[buf->write_index], data, first_chunk);

    // Copy second chunk if wraparound is needed
    uint32_t second_chunk = len - first_chunk;
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
        *ptr = NULL;
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

bool ringbuf_advance(ringbuf_t* buf, const uint32_t nbr_of_bytes)
{
    if (nbr_of_bytes > buf->items) return false;

    buf->read_index = (buf->read_index + nbr_of_bytes) % buf->size;
    buf->items -= (nbr_of_bytes > buf->items) ? buf->items : nbr_of_bytes;
    return true;
}


// internal helper: peek contiguous free space for writing
static uint32_t ringbuf_peek_write(const ringbuf_t* buf, uint8_t** ptr)
{
    if (buf->items == buf->size) {
        *ptr = NULL;
        return 0; // full
    }

    uint16_t wi = buf->write_index;
    uint16_t ri = buf->read_index;

    if (wi >= ri) {
        // Space until end of buffer
        uint32_t space = buf->size - wi;
        *ptr = &buf->data[wi];
        return space;
    } else {
        // Space is between write_index and read_index
        uint32_t space = ri - wi;
        *ptr = &buf->data[wi];
        return space;
    }
}

static void ringbuf_advance_write(ringbuf_t* buf, uint32_t n)
{
    buf->write_index = (buf->write_index + n) % buf->size;
    buf->items += n;
}

// ---------------------------------------------------------------------

bool ringbuf_consume(ringbuf_t* buf, ringbuf_consumer_fn consumer, void* ctx, const uint32_t max_bytes)
{
    uint32_t total = 0;

    uint8_t* ptr;
    uint32_t available = ringbuf_peek(buf, &ptr);

    while (available > 0) {
        uint32_t chunk = available;

        // obey max_bytes if >0
        if (max_bytes > 0 && (total + chunk) > max_bytes) {
            chunk = max_bytes - total;
        }

        int consumed = consumer(ctx, ptr, chunk);
        if (consumed <= 0) break; // backend didn’t accept data

        ringbuf_advance(buf, consumed);
        total += consumed;

        if (max_bytes > 0 && total >= max_bytes) break;

        available = ringbuf_peek(buf, &ptr);
    }

    return total > 0;
}

bool ringbuf_produce(ringbuf_t* buf, ringbuf_producer_fn producer, void* ctx, const uint32_t max_bytes)
{
    uint32_t total = 0;

    uint8_t* ptr;
    uint32_t space = ringbuf_peek_write(buf, &ptr);

    while (space > 0) {
        uint32_t chunk = space;

        // obey max_bytes if >0
        if (max_bytes > 0 && (total + chunk) > max_bytes) {
            chunk = max_bytes - total;
        }

        int produced = producer(ctx, ptr, chunk);
        if (produced <= 0) break; // backend had nothing to give

        ringbuf_advance_write(buf, produced);
        total += produced;

        if (max_bytes > 0 && total >= max_bytes) break;

        space = ringbuf_peek_write(buf, &ptr);
    }

    return total > 0;
}
