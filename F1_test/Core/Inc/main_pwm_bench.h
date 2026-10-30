#ifndef MAIN_PWM_BENCH_H
#define MAIN_PWM_BENCH_H
#include "main_pwm.h"
typedef enum {
    MAIN_PWM_TEST_IDLE, MAIN_PWM_TEST_RUNNING, MAIN_PWM_TEST_STOPPED,
    MAIN_PWM_TEST_DONE, MAIN_PWM_TEST_INVALID, MAIN_PWM_TEST_DRIVER_ERROR
} main_pwm_test_result_t;
typedef struct {
    uint32_t command;
    uint32_t main_channel;
    uint32_t pulse_us;
    uint32_t duration_ms;
    uint32_t sequence; /* Commit last. */
    uint32_t acknowledged_sequence;
    main_pwm_test_result_t result;
} main_pwm_test_mailbox_t;
extern volatile main_pwm_test_mailbox_t g_main_pwm_test;
void MainPwmBench_Init(void);
void MainPwmBench_Service(uint32_t now_ms);
#endif
