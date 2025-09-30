#include "unity.h"

void setUp() {}
void tearDown() {}

extern void test_rinbug_a();

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_rinbug_a);
    return UNITY_END();
}