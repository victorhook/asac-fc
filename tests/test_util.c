
#include "unity.h"
#include "util.h"   // your utils header

#define FLOAT_TOL 1e-6f

void setUp(void) {}
void tearDown(void) {}

/* --- within() tests --- */

void test_within_true(void)
{
    TEST_ASSERT_TRUE(within(5, 0, 10));
    TEST_ASSERT_TRUE(withinf(5.0f, 0.0f, 10.0f));
}

void test_within_false(void)
{
    TEST_ASSERT_FALSE(within(11, 0, 10));
    TEST_ASSERT_FALSE(withinf(-1.0f, 0.0f, 10.0f));
}

void test_within_boundary(void)
{
    TEST_ASSERT_TRUE(within(0, 0, 10));
    TEST_ASSERT_TRUE(within(10, 0, 10));
    TEST_ASSERT_TRUE(withinf(0.0f, 0.0f, 10.0f));
    TEST_ASSERT_TRUE(withinf(10.0f, 0.0f, 10.0f));
}

/* --- constrain() tests --- */

void test_constrain_inside_range(void)
{
    TEST_ASSERT_EQUAL(5, constrain(5, 0, 10));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 5.0f, constrainf(5.0f, 0.0f, 10.0f));
}

void test_constrain_below(void)
{
    TEST_ASSERT_EQUAL(0, constrain(-5, 0, 10));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, constrainf(-5.0f, 0.0f, 10.0f));
}

void test_constrain_above(void)
{
    TEST_ASSERT_EQUAL(10, constrain(15, 0, 10));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 10.0f, constrainf(15.0f, 0.0f, 10.0f));
}

/* --- map() tests --- */

void test_map_linear(void)
{
    TEST_ASSERT_EQUAL(50, map(5, 0, 10, 0, 100));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 50.0f, mapf(0.5f, 0.0f, 1.0f, 0.0f, 100.0f));
}

void test_map_reverse(void)
{
    TEST_ASSERT_EQUAL(50, map(5, 0, 10, 100, 0));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 50.0f, mapf(0.5f, 0.0f, 1.0f, 100.0f, 0.0f));
}

void test_map_out_of_range(void)
{
    TEST_ASSERT_EQUAL(-50, map(-5, 0, 10, 0, 100));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, -50.0f, mapf(-0.5f, 0.0f, 1.0f, 0.0f, 100.0f));
}

/* --- min/max tests --- */

void test_min_max_int(void)
{
    TEST_ASSERT_EQUAL(3, min(3, 7));
    TEST_ASSERT_EQUAL(7, max(3, 7));
}

void test_min_max_float(void)
{
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 3.0f, minf(3.0f, 7.0f));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 7.0f, maxf(3.0f, 7.0f));
}

/* --- clamp tests --- */

void test_clampf_low(void)
{
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 5.0f, clampf_low(5.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, clampf_low(-5.0f, 0.0f));
}

void test_clampf_high(void)
{
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 5.0f, clampf_high(5.0f, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 10.0f, clampf_high(15.0f, 10.0f));
}

void test_clampf_range(void)
{
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 5.0f, clampf(5.0f, 0.0f, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, clampf(-5.0f, 0.0f, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 10.0f, clampf(15.0f, 0.0f, 10.0f));
}

void test_clamp_ints(void)
{
    TEST_ASSERT_EQUAL(5, clamp(5, 0, 10));
    TEST_ASSERT_EQUAL(0, clamp(-5, 0, 10));
    TEST_ASSERT_EQUAL(10, clamp(15, 0, 10));
}

void test_clamp_low_high(void)
{
    TEST_ASSERT_EQUAL(0, clamp_low(-5, 0));
    TEST_ASSERT_EQUAL(5, clamp_low(5, 0));
    TEST_ASSERT_EQUAL(10, clamp_high(15, 10));
    TEST_ASSERT_EQUAL(5, clamp_high(5, 10));
}

/* --- main runner --- */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_within_true);
    RUN_TEST(test_within_false);
    RUN_TEST(test_within_boundary);

    RUN_TEST(test_constrain_inside_range);
    RUN_TEST(test_constrain_below);
    RUN_TEST(test_constrain_above);

    RUN_TEST(test_map_linear);
    RUN_TEST(test_map_reverse);
    RUN_TEST(test_map_out_of_range);

    RUN_TEST(test_min_max_int);
    RUN_TEST(test_min_max_float);

    RUN_TEST(test_clampf_low);
    RUN_TEST(test_clampf_high);
    RUN_TEST(test_clampf_range);
    RUN_TEST(test_clamp_ints);
    RUN_TEST(test_clamp_low_high);
    return UNITY_END();
}
