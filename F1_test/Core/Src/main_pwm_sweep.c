#include "main_pwm_sweep.h"
#if !F1_TEST_WATCH_MODE
#include "main_pwm_port.h"
#include <stdbool.h>
#include <string.h>

volatile main_pwm_sweep_status_t g_main_pwm_sweep;
static bool running;

static void apply(uint32_t now_ms, uint32_t duty)
{
    uint16_t pulse[4] = {0U, 0U, 0U, 0U};
    pulse[0] = (uint16_t)(MAIN_PWM_PERIOD_US * duty / 100U);
    pulse[1] = pulse[2] = pulse[3] = pulse[0];
    /* Bypass the ESC API only inside this separate scope-test target.
     * Same channel mask preserves the timer phase on subsequent steps. */
    MainPwmPort_Apply(0x1EU, pulse);
    g_main_pwm_sweep.duty_percent = duty;
    g_main_pwm_sweep.pulse_us = pulse[0];
    g_main_pwm_sweep.last_step_ms = now_ms;
    running = true;
}

void MainPwmSweep_Init(uint32_t now_ms)
{
    memset((void *)&g_main_pwm_sweep, 0, sizeof(g_main_pwm_sweep));
    g_main_pwm_sweep.magic = 0x53575031UL;
    g_main_pwm_sweep.enabled = 1U;
    apply(now_ms, 10U);
}

void MainPwmSweep_Service(uint32_t now_ms)
{
    if (g_main_pwm_sweep.enabled != 1U) {
        if (running) { MainPwmPort_Stop(); running = false; }
        return;
    }
    if (!running) { apply(now_ms, 10U); }
    else if ((uint32_t)(now_ms - g_main_pwm_sweep.last_step_ms) >= 1000U) {
        uint32_t duty = g_main_pwm_sweep.duty_percent;
        duty = duty >= 90U ? 10U : duty + 10U;
        apply(now_ms, duty);
        ++g_main_pwm_sweep.transitions;
    }
}
#endif
