
#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t* data;
    uint16_t size;
    uint16_t write_index;
    uint16_t read_index;
    uint16_t items;
} ringbuf_t;

/** Initializes the ringbuffer */
void ringbuf_init(ringbuf_t* buf, uint8_t* data, const uint32_t size);

/** Returns a byte from the byte buffer. Returns true if successful (eg it was not empty) */
bool ringbuf_get(ringbuf_t* buf, uint8_t* byte);

/** Adds a byte to the ringbuffer. If the buffer is full the byte is discarded. Returns true if successful. */
bool ringbuf_add(ringbuf_t* buf, const uint8_t byte);

/** Adds number of bytes to the ringbuffer. If the buffer is full the bytes are discarded. Returns true if successful. */
bool ringbuf_add_bytes(ringbuf_t* buf, const uint8_t* data, const uint16_t len);

/** 
    Returns a number containing the longest contiguous block and sets ptr to the internal raw buffer.
    This can be useful when needing to write data directly from the internal buffer without doing extra memcpy.
*/
uint32_t ringbuf_peek(const ringbuf_t* buf, uint8_t** ptr);

/** Advances the read buffer index with `nbr_of_bytes` */
void ringbuf_advance(ringbuf_t* buf, const uint32_t nbr_of_bytes);


/** Returns the number of items in the ringbuffer */
static inline int ringbuf_items(const ringbuf_t* buf)
{
    return buf->items;
}

/** Returns the number of free space available in the ringbuffer */
static inline int ringbuf_free_space(const ringbuf_t* buf)
{
    return buf->size - buf->items;
}

/** Returns true if the buffer is empty */
static inline bool ringbuf_is_empty(const ringbuf_t* buf)
{
    return buf->items == 0;
}

/** Returns true if the buffer is full */
static inline bool ringbuf_is_full(const ringbuf_t* buf)
{
    return buf->items == buf->size;
}


#endif
