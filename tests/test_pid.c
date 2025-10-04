#include "unity.h"
#include "pid.h"
#include <stdbool.h>
#include <math.h>

#define FLOAT_TOL 1e-5f

static pid_handle_t pid;

void setUp(void)
{
    pid_reset(&pid);
    pid.imax = 10.0f;
}

void tearDown(void) {}

/* --- Tests --- */

// Reset should clear state
void test_pid_reset_clears_state(void)
{
    pid.error = 3.0f;
    pid.i = 5.0f;
    pid.out = 99.0f;

    pid_reset(&pid);

    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, pid.error);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, pid.i);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, pid.out);
}

// Pure proportional
void test_pid_proportional_term(void)
{
    pid.Kp = 2.0f;  // double gain
    float out = pid_update(&pid, 5.0f, 0.0f, false, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 10.0f, out);
}

// Integral accumulates
void test_pid_integral_accumulates(void)
{
    pid.Ki = 1.0f;
    float out1 = pid_update(&pid, 2.0f, 0.0f, false, 1.0f);
    float out2 = pid_update(&pid, 2.0f, 0.0f, false, 1.0f);
    TEST_ASSERT_TRUE(out2 > out1);
}

// Integral clamp
void test_pid_integral_clamped(void)
{
    pid.Ki = 10.0f;
    pid.imax = 1.0f;
    for (int i = 0; i < 100; i++) {
        pid_update(&pid, 1.0f, 0.0f, false, 1.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 1.0f, pid.i);
}

// Skip integrator
void test_pid_skip_integrator(void)
{
    pid.Ki = 1.0f;
    pid_update(&pid, 5.0f, 0.0f, true, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, pid.i);
}

// Derivative response to error change
void test_pid_derivative_term(void)
{
    pid.Kd = 1.0f;
    pid_update(&pid, 5.0f, 0.0f, false, 1.0f); // first call
    float out2 = pid_update(&pid, 10.0f, 0.0f, false, 1.0f); // error jump
    TEST_ASSERT_TRUE(fabsf(out2) > 0.0f);
}

// Derivative goes to zero when error stable
void test_pid_derivative_zero_on_constant_error(void)
{
    pid.Kp = 0.0f;
    pid.Ki = 0.0f;
    pid.Kd = 1.0f;

    pid_update(&pid, 5.0f, 0.0f, false, 1.0f); // first call: error jump
    float out2 = pid_update(&pid, 5.0f, 0.0f, false, 1.0f); // error stable
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, out2);
}

// Feedforward
void test_pid_feedforward(void)
{
    pid.Kff = 2.0f;
    float out = pid_update(&pid, 3.0f, 0.0f, false, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 6.0f, pid.ff);
    TEST_ASSERT_TRUE(fabsf(out - 6.0f) < 20.0f); // ff should contribute
}

// Mixed P+I+D
void test_pid_combined_pid(void)
{
    pid.Kp = 1.0f;
    pid.Ki = 0.5f;
    pid.Kd = 0.1f;

    float out1 = pid_update(&pid, 5.0f, 0.0f, false, 1.0f);
    float out2 = pid_update(&pid, 5.0f, 0.0f, false, 1.0f);

    // out2 should be bigger than out1 due to integrator growth
    TEST_ASSERT_TRUE(out2 > out1);
}

// dt scaling
void test_pid_dt_effect(void)
{
    pid.Kp = 0.0f;
    pid.Kd = 0.0f;
    pid.Ki = 1.0f;

    // Run with dt = 1.0
    pid_reset(&pid);
    float out1 = pid_update(&pid, 5.0f, 0.0f, false, 1.0f);

    // Run with dt = 0.1
    pid_reset(&pid);
    float out2 = pid_update(&pid, 5.0f, 0.0f, false, 0.1f);

    // Expect smaller dt to produce smaller immediate integral contribution
    TEST_ASSERT_TRUE(out2 < out1);
}

// Zero gains → always zero output
void test_pid_zero_gains_gives_zero_output(void)
{
    pid.Kp = 0.0f;
    pid.Ki = 0.0f;
    pid.Kd = 0.0f;
    pid.Kff = 0.0f;

    float out = pid_update(&pid, 10.0f, 0.0f, false, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, out);
}

// Negative error → negative P output
void test_pid_negative_error(void)
{
    pid.Kp = 2.0f;
    pid.Ki = 0.0f;
    pid.Kd = 0.0f;

    float out = pid_update(&pid, 0.0f, 10.0f, false, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, -20.0f, out);
}

// Very small dt should not explode if error is constant
void test_pid_small_dt_no_derivative_spike_on_constant_error(void)
{
    pid.Kp = 0.0f;
    pid.Ki = 0.0f;
    pid.Kd = 1.0f;

    pid_update(&pid, 5.0f, 0.0f, false, 1.0f); // initialize prev_error
    float out = pid_update(&pid, 5.0f, 0.0f, false, 0.0001f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, out);
}

// Large error but integrator should respect imax
void test_pid_large_error_integrator_clamped(void)
{
    pid.Kp = 0.0f;
    pid.Ki = 5.0f;
    pid.Kd = 0.0f;
    pid.imax = 2.0f;

    for (int i = 0; i < 100; i++) {
        pid_update(&pid, 1000.0f, 0.0f, false, 1.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 2.0f, pid.i);
}

// Feedforward only, error = 0
void test_pid_feedforward_with_zero_error(void)
{
    pid.Kp = 0.0f;
    pid.Ki = 0.0f;
    pid.Kd = 0.0f;
    pid.Kff = 1.0f;

    float out = pid_update(&pid, 7.0f, 7.0f, false, 1.0f); // target=actual
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 7.0f, out);
}

// Multiple updates → derivative should react only to changes
void test_pid_derivative_on_step_change_only(void)
{
    pid.Kp = 0.0f;
    pid.Ki = 0.0f;
    pid.Kd = 1.0f;
    pid.Kff = 0.0;

    // First step: error jump from 0 -> 5
    float out1 = pid_update(&pid, 5.0f, 0.0f, false, 1.0f);
    TEST_ASSERT_TRUE(out1 > 0.0f);

    // Hold error constant
    float out2 = pid_update(&pid, 5.0f, 0.0f, false, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, out2);

    // Step error again
    float out3 = pid_update(&pid, 10.0f, 0.0f, false, 1.0f);
    TEST_ASSERT_TRUE(out3 > 0.0f);
}

// Test that imax=0 disables integration completely
void test_pid_imax_zero_disables_integration(void)
{
    pid.Kp = 0.0f;
    pid.Ki = 1.0f;
    pid.Kd = 0.0f;
    pid.imax = 0.0f;

    pid_update(&pid, 5.0f, 0.0f, false, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 0.0f, pid.i);
}

// Calling update with dt=0 should not crash (could clamp internally)
void test_pid_dt_zero_safe(void)
{
    pid.Kp = 0.0f;
    pid.Ki = 1.0f;
    pid.Kd = 0.0f;
    pid.Kff = 0;
    float out = pid_update(&pid, 5.0f, 0.0f, false, 0.0f);
    // Depending on your implementation, you may clamp dt or ignore D term
    TEST_ASSERT_TRUE(isfinite(out));
}


int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_pid_reset_clears_state);
    RUN_TEST(test_pid_proportional_term);
    RUN_TEST(test_pid_integral_accumulates);
    RUN_TEST(test_pid_integral_clamped);
    RUN_TEST(test_pid_skip_integrator);
    RUN_TEST(test_pid_derivative_term);
    RUN_TEST(test_pid_derivative_zero_on_constant_error);
    RUN_TEST(test_pid_feedforward);
    RUN_TEST(test_pid_combined_pid);
    RUN_TEST(test_pid_dt_effect);
    RUN_TEST(test_pid_zero_gains_gives_zero_output);
    RUN_TEST(test_pid_negative_error);
    RUN_TEST(test_pid_small_dt_no_derivative_spike_on_constant_error);
    RUN_TEST(test_pid_large_error_integrator_clamped);
    RUN_TEST(test_pid_feedforward_with_zero_error);
    RUN_TEST(test_pid_derivative_on_step_change_only);
    RUN_TEST(test_pid_imax_zero_disables_integration);
    RUN_TEST(test_pid_dt_zero_safe);
    return UNITY_END();
}
