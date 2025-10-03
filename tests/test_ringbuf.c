#include "unity.h"
#include "ringbuf.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define BUFSZ 16
#define ASSERT_BYTES_EQ(exp, act, n) TEST_ASSERT_EQUAL_UINT8_ARRAY((exp), (act), (n))

static ringbuf_t rb;
static uint8_t storage[BUFSZ];

void setUp(void)
{
    memset(storage, 0x00, sizeof(storage));
    ringbuf_init(&rb, storage, BUFSZ);
}

void tearDown(void) {}

/* -------------------- Helpers & Mocks -------------------- */

typedef struct {
    uint8_t *sink;
    uint32_t cap;
    uint32_t written;
    uint32_t calls;
} consumer_ctx_t;

/* consumer copies out data and reports "consumed bytes" */
static int mock_consumer(void* vctx, uint8_t* data, const uint32_t len)
{
    consumer_ctx_t* ctx = (consumer_ctx_t*)vctx;
    const uint32_t to_copy = (ctx->written + len <= ctx->cap) ? len : (ctx->cap - ctx->written);
    if (to_copy > 0) memcpy(ctx->sink + ctx->written, data, to_copy);
    ctx->written += to_copy;
    ctx->calls++;
    return (int)to_copy;
}

typedef struct {
    const uint8_t *src;
    uint32_t total;
    uint32_t pos;
    uint32_t calls;
} producer_ctx_t;

/* producer writes INTO the buffer pointer it is given.
   NOTE: header uses const uint8_t* for 'data' (dest) — we cast away const here for tests. */
static int mock_producer(void* vctx, const uint8_t* data, const uint32_t len)
{
    producer_ctx_t* ctx = (producer_ctx_t*)vctx;
    const uint32_t remaining = ctx->total - ctx->pos;
    const uint32_t to_copy = (remaining < len) ? remaining : len;
    if (to_copy > 0) {
        memcpy((uint8_t*)data, ctx->src + ctx->pos, to_copy);
        ctx->pos += to_copy;
    }
    ctx->calls++;
    return (int)to_copy;
}

/* -------------------- Tests: Init & Basic Properties -------------------- */

void test_init_sets_empty_and_indices_zero(void)
{
    TEST_ASSERT_EQUAL_PTR(storage, rb.data);
    TEST_ASSERT_EQUAL_UINT16(BUFSZ, rb.size);
    TEST_ASSERT_EQUAL_UINT16(0, rb.items);
    TEST_ASSERT_EQUAL_UINT16(0, rb.read_index);
    TEST_ASSERT_EQUAL_UINT16(0, rb.write_index);
    TEST_ASSERT_TRUE(ringbuf_is_empty(&rb));
    TEST_ASSERT_FALSE(ringbuf_is_full(&rb));
    TEST_ASSERT_EQUAL(BUFSZ, ringbuf_free_space(&rb));
}

void test_read_byte_empty_returns_false_and_no_modify(void)
{
    uint8_t b = 0xAA;
    bool ok = ringbuf_read_byte(&rb, &b);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_UINT8(0xAA, b);
    TEST_ASSERT_EQUAL(0, ringbuf_items(&rb));
}

/* -------------------- Single-Byte R/W -------------------- */

void test_write_byte_then_read_byte_roundtrip(void)
{
    TEST_ASSERT_TRUE(ringbuf_write_byte(&rb, 0x42));
    TEST_ASSERT_EQUAL(1, ringbuf_items(&rb));

    uint8_t b = 0;
    TEST_ASSERT_TRUE(ringbuf_read_byte(&rb, &b));
    TEST_ASSERT_EQUAL_UINT8(0x42, b);
    TEST_ASSERT_EQUAL(0, ringbuf_items(&rb));
}

void test_fill_to_capacity_is_full_and_further_write_byte_fails(void)
{
    for (int i = 0; i < BUFSZ; i++) {
        TEST_ASSERT_TRUE(ringbuf_write_byte(&rb, (uint8_t)i));
    }
    TEST_ASSERT_TRUE(ringbuf_is_full(&rb));
    TEST_ASSERT_EQUAL(0, ringbuf_free_space(&rb));
    TEST_ASSERT_FALSE(ringbuf_write_byte(&rb, 0xEE)); // full → discard
    TEST_ASSERT_EQUAL(BUFSZ, ringbuf_items(&rb));     // unchanged
}

/* -------------------- Multi-Byte R/W (happy path) -------------------- */

void test_write_and_read_exact_length(void)
{
    uint8_t in[8]; for (int i=0;i<8;i++) in[i]=(uint8_t)(i+1);
    TEST_ASSERT_TRUE(ringbuf_write(&rb, in, 8));
    TEST_ASSERT_EQUAL(8, ringbuf_items(&rb));

    uint8_t out[8]={0};
    TEST_ASSERT_TRUE(ringbuf_read(&rb, out, 8));
    ASSERT_BYTES_EQ(in, out, 8);
    TEST_ASSERT_EQUAL(0, ringbuf_items(&rb));
}

/* -------------------- Wrap-around & Order -------------------- */

void test_wraparound_preserves_fifo_order(void)
{
    uint8_t first[10]; for (int i=0;i<10;i++) first[i]=(uint8_t)(0xA0+i);
    TEST_ASSERT_TRUE(ringbuf_write(&rb, first, 10));
    // read 6 to move read_index forward
    uint8_t tmp[6]={0};
    TEST_ASSERT_TRUE(ringbuf_read(&rb, tmp, 6));
    ASSERT_BYTES_EQ(first, tmp, 6);

    // write 8 more to wrap
    uint8_t second[8]; for (int i=0;i<8;i++) second[i]=(uint8_t)(0xB0+i);
    TEST_ASSERT_TRUE(ringbuf_write(&rb, second, 8));

    // now read remaining 4 of first + all 8 of second
    uint8_t out[12]={0};
    TEST_ASSERT_TRUE(ringbuf_read(&rb, out, 12));
    ASSERT_BYTES_EQ(first+6, out, 4);
    ASSERT_BYTES_EQ(second, out+4, 8);
    TEST_ASSERT_EQUAL(0, ringbuf_items(&rb));
}

/* -------------------- Peek & Advance -------------------- */

void test_peek_no_wrap_returns_entire_items(void)
{
    uint8_t in[5]; for (int i=0;i<5;i++) in[i]=(uint8_t)(10+i);
    TEST_ASSERT_TRUE(ringbuf_write(&rb, in, 5));
    uint8_t* ptr = NULL;
    uint32_t n = ringbuf_peek(&rb, &ptr);
    TEST_ASSERT_EQUAL_UINT32(5, n);
    TEST_ASSERT_EQUAL_PTR(storage + rb.read_index, ptr);
    ASSERT_BYTES_EQ(in, ptr, 5);
}

void test_peek_when_wrapped_stops_at_end_of_storage(void)
{
    uint8_t a[12]; for (int i=0;i<12;i++) a[i]=(uint8_t)(i);
    TEST_ASSERT_TRUE(ringbuf_write(&rb, a, 12));
    uint8_t tmp[10];
    TEST_ASSERT_TRUE(ringbuf_read(&rb, tmp, 10)); // read_index=10, write_index=12
    // write 8 → will wrap
    uint8_t b[8]; for (int i=0;i<8;i++) b[i]=(uint8_t)(0x80+i);
    TEST_ASSERT_TRUE(ringbuf_write(&rb, b, 8));

    // Now peek should give contiguous from read_index to end
    uint8_t* ptr = NULL;
    uint32_t n = ringbuf_peek(&rb, &ptr);
    TEST_ASSERT_EQUAL_UINT32(BUFSZ - rb.read_index, n); // until end
    TEST_ASSERT_EQUAL_PTR(storage + rb.read_index, ptr);
}

void test_advance_valid_and_invalid(void)
{
    uint8_t in[6]; for (int i=0;i<6;i++) in[i]=(uint8_t)(0x30+i);
    TEST_ASSERT_TRUE(ringbuf_write(&rb, in, 6));
    TEST_ASSERT_TRUE(ringbuf_advance(&rb, 3));
    TEST_ASSERT_EQUAL(3, ringbuf_items(&rb)); // consumed 3

    // Advancing more than items should fail and not change state
    uint16_t prev_items = (uint16_t)ringbuf_items(&rb);
    uint16_t prev_r = rb.read_index, prev_w = rb.write_index;
    TEST_ASSERT_FALSE(ringbuf_advance(&rb, 10));
    TEST_ASSERT_EQUAL(prev_items, ringbuf_items(&rb));
    TEST_ASSERT_EQUAL(prev_r, rb.read_index);
    TEST_ASSERT_EQUAL(prev_w, rb.write_index);
}

void test_peek_empty_returns_zero(void)
{
    uint8_t* ptr = (uint8_t*)0xDEADBEEF;
    uint32_t n = ringbuf_peek(&rb, &ptr);
    TEST_ASSERT_EQUAL_UINT32(0, n);
    // ptr is don't-care when empty; ensure no crash, no deref.
}

/* -------------------- Consume (drain via callback) -------------------- */

void test_consume_drains_all_when_max_zero(void)
{
    uint8_t in[11]; for (int i=0;i<11;i++) in[i]=(uint8_t)(i+1);
    TEST_ASSERT_TRUE(ringbuf_write(&rb, in, 11));

    uint8_t sink[32]={0};
    consumer_ctx_t ctx = { .sink=sink, .cap=sizeof(sink), .written=0, .calls=0 };
    TEST_ASSERT_TRUE(ringbuf_consume(&rb, mock_consumer, &ctx, 0)); // 0 => all

    TEST_ASSERT_EQUAL(11, (int)ctx.written);
    ASSERT_BYTES_EQ(in, sink, 11);
    TEST_ASSERT_EQUAL(0, ringbuf_items(&rb));
}

void test_consume_respects_max_bytes(void)
{
    uint8_t in[12]; for (int i=0;i<12;i++) in[i]=(uint8_t)(0x40+i);
    TEST_ASSERT_TRUE(ringbuf_write(&rb, in, 12));

    uint8_t sink[32]={0};
    consumer_ctx_t ctx = { .sink=sink, .cap=sizeof(sink), .written=0, .calls=0 };
    TEST_ASSERT_TRUE(ringbuf_consume(&rb, mock_consumer, &ctx, 7));

    TEST_ASSERT_EQUAL(7, (int)ctx.written);
    ASSERT_BYTES_EQ(in, sink, 7);
    TEST_ASSERT_EQUAL(12-7, ringbuf_items(&rb));
}

/* -------------------- Produce (fill via callback) -------------------- */

void test_produce_fills_until_full_when_max_zero(void)
{
    uint8_t src[64]; for (int i=0;i<64;i++) src[i]=(uint8_t)(0x90+i);
    producer_ctx_t pctx = { .src=src, .total=64, .pos=0, .calls=0 };
    TEST_ASSERT_TRUE(ringbuf_produce(&rb, mock_producer, &pctx, 0)); // 0 => as much as fits

    TEST_ASSERT_TRUE(ringbuf_is_full(&rb));
    TEST_ASSERT_EQUAL(BUFSZ, ringbuf_items(&rb));

    // Verify content order by draining
    uint8_t out[BUFSZ]={0};
    TEST_ASSERT_TRUE(ringbuf_read(&rb, out, BUFSZ));
    ASSERT_BYTES_EQ(src, out, BUFSZ);
}

void test_produce_respects_max_bytes(void)
{
    uint8_t src[32]; for (int i=0;i<32;i++) src[i]=(uint8_t)(0x50+i);
    producer_ctx_t pctx = { .src=src, .total=32, .pos=0, .calls=0 };
    TEST_ASSERT_TRUE(ringbuf_produce(&rb, mock_producer, &pctx, 9));
    TEST_ASSERT_EQUAL(9, ringbuf_items(&rb));

    uint8_t out[9]={0};
    TEST_ASSERT_TRUE(ringbuf_read(&rb, out, 9));
    ASSERT_BYTES_EQ(src, out, 9);
}

/* -------------------- Edge Cases & No-ops -------------------- */

void test_write_zero_length_is_noop_and_true(void)
{
    uint16_t items_before = (uint16_t)ringbuf_items(&rb);
    TEST_ASSERT_TRUE(ringbuf_write(&rb, (const uint8_t*)"ignored", 0));
    TEST_ASSERT_EQUAL(items_before, ringbuf_items(&rb));
}

void test_read_zero_length_is_noop_and_true(void)
{
    // Put something in, but read 0
    TEST_ASSERT_TRUE(ringbuf_write_byte(&rb, 0xAB));
    uint16_t before = (uint16_t)ringbuf_items(&rb);
    TEST_ASSERT_TRUE(ringbuf_read(&rb, storage, 0));
    TEST_ASSERT_EQUAL(before, ringbuf_items(&rb));
}

void test_advance_zero_is_noop_and_true(void)
{
    TEST_ASSERT_TRUE(ringbuf_write_byte(&rb, 0x11));
    uint16_t r = rb.read_index, w = rb.write_index, items = (uint16_t)ringbuf_items(&rb);
    TEST_ASSERT_TRUE(ringbuf_advance(&rb, 0));
    TEST_ASSERT_EQUAL(r, rb.read_index);
    TEST_ASSERT_EQUAL(w, rb.write_index);
    TEST_ASSERT_EQUAL(items, ringbuf_items(&rb));
}

/* -------------------- Full Buffer Behavior -------------------- */

void test_write_when_full_multi_returns_false_and_discards(void)
{
    // Fill completely
    uint8_t fill[BUFSZ]; for (int i=0;i<BUFSZ;i++) fill[i]=(uint8_t)i;
    TEST_ASSERT_TRUE(ringbuf_write(&rb, fill, BUFSZ));
    TEST_ASSERT_TRUE(ringbuf_is_full(&rb));

    uint8_t extra[3] = {0xEE,0xEF,0xF0};
    TEST_ASSERT_FALSE(ringbuf_write(&rb, extra, 3)); // spec: discard when full
    TEST_ASSERT_TRUE(ringbuf_is_full(&rb));

    // Drain and verify original data intact
    uint8_t out[BUFSZ]={0};
    TEST_ASSERT_TRUE(ringbuf_read(&rb, out, BUFSZ));
    ASSERT_BYTES_EQ(fill, out, BUFSZ);
}

/* -------------------- Invariants & Stress-ish -------------------- */

void test_items_plus_free_space_equals_size_over_ops(void)
{
    // write 7
    uint8_t a[7]; for (int i=0;i<7;i++) a[i]=(uint8_t)(i+1);
    TEST_ASSERT_TRUE(ringbuf_write(&rb, a, 7));
    TEST_ASSERT_EQUAL(BUFSZ, ringbuf_items(&rb) + ringbuf_free_space(&rb));

    // read 3
    uint8_t tmp[3];
    TEST_ASSERT_TRUE(ringbuf_read(&rb, tmp, 3));
    TEST_ASSERT_EQUAL(BUFSZ, ringbuf_items(&rb) + ringbuf_free_space(&rb));

    // write 10 (will fill)
    uint8_t b[10]; for (int i=0;i<10;i++) b[i]=(uint8_t)(0x70+i);
    const bool wrote = ringbuf_write(&rb, b, 10);
    TEST_ASSERT_TRUE(wrote);
    TEST_ASSERT_EQUAL(BUFSZ, ringbuf_items(&rb) + ringbuf_free_space(&rb));
}

void test_fifo_order_under_many_small_ops(void)
{
    uint8_t ref[BUFSZ*3];
    for (int i=0;i<(int)sizeof(ref); i++) ref[i]=(uint8_t)(0xC0 + (i & 0x3F));
    int wpos=0, rpos=0;

    // push & pop in varying chunk sizes
    for (int i=0;i<48; i++) {
        int wlen = (i%5)+1; // 1..5
        int rlen = (i%3);   // 0..2
        if (wpos + wlen <= (int)sizeof(ref)) {
            bool ok = ringbuf_write(&rb, &ref[wpos], (uint32_t)wlen);
            if (ok) {
                wpos += wlen;
            }
        }
        if (rlen>0 && ringbuf_items(&rb) >= rlen) {
            uint8_t out[5]={0};
            TEST_ASSERT_TRUE(ringbuf_read(&rb, out, (uint32_t)rlen));
            ASSERT_BYTES_EQ(&ref[rpos], out, rlen);
            rpos += rlen;
        }
        TEST_ASSERT_EQUAL(BUFSZ, ringbuf_items(&rb) + ringbuf_free_space(&rb));
    }

    // drain remaining
    uint8_t remain[BUFSZ*2]={0};
    int remaining = ringbuf_items(&rb);
    if (remaining > 0) {
        TEST_ASSERT_TRUE(ringbuf_read(&rb, remain, (uint32_t)remaining));
        ASSERT_BYTES_EQ(&ref[rpos], remain, remaining);
        rpos += remaining;
    }
    TEST_ASSERT_EQUAL(wpos, rpos);
    TEST_ASSERT_TRUE(ringbuf_is_empty(&rb));
}

/* -------------------- Main Runner -------------------- */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_init_sets_empty_and_indices_zero);
    RUN_TEST(test_read_byte_empty_returns_false_and_no_modify);

    RUN_TEST(test_write_byte_then_read_byte_roundtrip);
    RUN_TEST(test_fill_to_capacity_is_full_and_further_write_byte_fails);

    RUN_TEST(test_write_and_read_exact_length);

    RUN_TEST(test_wraparound_preserves_fifo_order);

    RUN_TEST(test_peek_no_wrap_returns_entire_items);
    RUN_TEST(test_peek_when_wrapped_stops_at_end_of_storage);
    RUN_TEST(test_advance_valid_and_invalid);
    RUN_TEST(test_peek_empty_returns_zero);

    RUN_TEST(test_consume_drains_all_when_max_zero);
    RUN_TEST(test_consume_respects_max_bytes);

    RUN_TEST(test_produce_fills_until_full_when_max_zero);
    RUN_TEST(test_produce_respects_max_bytes);

    RUN_TEST(test_write_zero_length_is_noop_and_true);
    RUN_TEST(test_read_zero_length_is_noop_and_true);
    RUN_TEST(test_advance_zero_is_noop_and_true);

    RUN_TEST(test_write_when_full_multi_returns_false_and_discards);

    RUN_TEST(test_items_plus_free_space_equals_size_over_ops);
    RUN_TEST(test_fifo_order_under_many_small_ops);

    return UNITY_END();
}
