
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

/** Function to write data directly from the underlying buffer */
typedef int (*ringbuf_producer_fn)(void* ctx, const uint8_t* data, const uint32_t len);

/** Function to read data directly from the underlying buffer calling ringbuf_consumer_fn(data, len) */
typedef int (*ringbuf_consumer_fn)(void* ctx, uint8_t* data, const uint32_t len);

/** Initializes the ringbuffer */
void ringbuf_init(ringbuf_t* buf, uint8_t* data, const uint32_t size);

/** Returns a byte from the byte buffer. Returns true if successful (eg it was not empty) */
bool ringbuf_read_byte(ringbuf_t* buf, uint8_t* byte);

/** Returns `len` bytes from the byte buffer. Returns true if all bytes read */
bool ringbuf_read(ringbuf_t* buf, uint8_t* data, const uint32_t len);

/** Adds a byte to the ringbuffer. If the buffer is full the byte is discarded. Returns true if successful. */
bool ringbuf_write_byte(ringbuf_t* buf, const uint8_t byte);

/** Adds number of bytes to the ringbuffer. If the buffer is full the bytes are discarded. Returns true if successful. */
bool ringbuf_write(ringbuf_t* buf, const uint8_t* data, const uint32_t len);

/** 
    Returns a number containing the longest contiguous block and sets ptr to the internal raw buffer.
    This can be useful when needing to write data directly from the internal buffer without doing extra memcpy.
*/
uint32_t ringbuf_peek(const ringbuf_t* buf, uint8_t** ptr);

/** Advances the read buffer index with `nbr_of_bytes` */
bool ringbuf_advance(ringbuf_t* buf, const uint32_t nbr_of_bytes);



/** Drains the ringbuffer (or max `max_bytes` if it's not `0`) by writing all data by calling function `ringbuf_consumer_fn`  */
bool ringbuf_consume(ringbuf_t* buf, ringbuf_consumer_fn consumer, void* ctx, const uint32_t max_bytes);

/** Fill the ringbuffer (or max `max_bytes` if it's not `0`) by reading data by calling function `ringbuf_producer_fn` */
bool ringbuf_produce(ringbuf_t* buf, ringbuf_producer_fn producer, void* ctx, const uint32_t max_bytes);


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
