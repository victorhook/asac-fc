#include "unity.h"
#include "util/ringbuf.h"


void test_rinbug_a()
{
    TEST_ASSERT_FLOAT_WITHIN(0.7, 1, 0.5);
}