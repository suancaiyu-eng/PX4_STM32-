#include "main_pwm_bench.h"
#include <string.h>
volatile main_pwm_test_mailbox_t g_main_pwm_test;
static uint32_t test_sequence, test_start_ms, test_duration_ms;
static bool test_running;
static bool test_continuous;
static main_pwm_output_t test_output;
static void bench_stop(main_pwm_stop_reason_t reason)
{
    MainPwm_Stop();
    g_main_pwm_status.stop_reason = reason;
}
void MainPwmBench_Init(void)
{
    memset((void *)&g_main_pwm_test, 0, sizeof(g_main_pwm_test));
    g_main_pwm_test.main_channel = 2U;
    g_main_pwm_test.pulse_us = 1000U;
    g_main_pwm_test.duration_ms = 1000U;
    test_sequence = 0U;
    test_running = test_continuous = false;
}
void MainPwmBench_Service(uint32_t now_ms)
{
    uint32_t sequence = g_main_pwm_test.sequence;
    if (sequence != test_sequence) {
        uint32_t delta = sequence - test_sequence;
        uint32_t command = g_main_pwm_test.command;
        uint32_t channel = g_main_pwm_test.main_channel;
        uint32_t pulse = g_main_pwm_test.pulse_us;
        uint32_t duration = g_main_pwm_test.duration_ms;
        /* Sequence must move forward by less than half the uint32 range.
         * Backward/replayed values neither execute nor consume a new lease. */
        if (delta < 0x80000000UL && sequence == g_main_pwm_test.sequence) {
            test_sequence = sequence;
            MainPwm_Stop();
            test_running = test_continuous = false;
            if (command == 2U) {
                g_main_pwm_test.result = MAIN_PWM_TEST_STOPPED;
            } else if ((command != 1U && command != 3U) || channel < 2U || channel > 5U ||
                       pulse < MAIN_PWM_MIN_US || pulse > MAIN_PWM_MAX_US ||
                       (command == 1U && (duration == 0U || duration > MAIN_PWM_TIMEOUT_MS))) {
                g_main_pwm_test.result = MAIN_PWM_TEST_INVALID;
            } else {
                main_pwm_output_t output;
                output.main_channel = (uint8_t)channel;
                output.pulse_us = (uint16_t)pulse;
                if (MainPwm_Submit(&output, 1U, now_ms)) {
                    test_running = true;
                    test_start_ms = now_ms;
                    test_duration_ms = duration;
                    test_continuous = command == 3U;
                    test_output = output; /* Watch edits require a NEW sequence. */
                    g_main_pwm_test.result = MAIN_PWM_TEST_RUNNING;
                } else { g_main_pwm_test.result = MAIN_PWM_TEST_DRIVER_ERROR; }
            }
            g_main_pwm_test.acknowledged_sequence = sequence;
        }
    }
    if (test_running && !test_continuous && (uint32_t)(now_ms - test_start_ms) >= test_duration_ms) {
        bench_stop(MAIN_PWM_STOP_TEST_DONE);
        test_running = false;
        g_main_pwm_test.result = MAIN_PWM_TEST_DONE;
    }
    if (test_running && g_main_pwm_status.active_mask == 0U) {
        test_running = false;
        test_continuous = false;
        g_main_pwm_test.result = MAIN_PWM_TEST_DRIVER_ERROR;
    }
    /* Explicit bench-only continuous mode. Renew the captured output at 10 Hz
     * without restarting the timer. A missed driver lease stops/rejects; never
     * automatically restart after a fault or a stalled foreground loop. */
    if (test_running && test_continuous &&
        (uint32_t)(now_ms - g_main_pwm_status.last_update_ms) >= 100U) {
        if (!MainPwm_Submit(&test_output, 1U, now_ms)) {
            bench_stop(MAIN_PWM_STOP_TIMEOUT);
            test_running = test_continuous = false;
            g_main_pwm_test.result = MAIN_PWM_TEST_DRIVER_ERROR;
        }
    }
}
