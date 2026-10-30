#include "main_pwm.h"
#include "main_pwm_port.h"
#include <stddef.h>
#include <string.h>

volatile main_pwm_status_t g_main_pwm_status;

static void stop(main_pwm_stop_reason_t reason)
{
    uint8_t i;
    MainPwmPort_Stop();
    g_main_pwm_status.active_mask = 0U;
    for (i = 0U; i < 4U; ++i) { g_main_pwm_status.pulse_us[i] = 0U; }
    g_main_pwm_status.stop_reason = reason;
}

bool MainPwm_Init(uint32_t now_ms)
{
    bool ok;
    memset((void *)&g_main_pwm_status, 0, sizeof(g_main_pwm_status));
    ok = MainPwmPort_Init();
    g_main_pwm_status.initialized = ok;
    g_main_pwm_status.last_update_ms = now_ms;
    stop(ok ? MAIN_PWM_STOP_EXPLICIT : MAIN_PWM_STOP_INIT_ERROR);
    return ok;
}

bool MainPwm_Submit(const main_pwm_output_t *outputs, uint8_t count, uint32_t now_ms)
{
    uint8_t mask = 0U, i;
    uint16_t pulse[4] = {0U, 0U, 0U, 0U};
    /* A late caller cannot silently extend an expired output lease. */
    if (g_main_pwm_status.active_mask != 0U &&
        (uint32_t)(now_ms - g_main_pwm_status.last_update_ms) >= MAIN_PWM_TIMEOUT_MS) {
        stop(MAIN_PWM_STOP_TIMEOUT);
        ++g_main_pwm_status.rejected_frames;
        return false;
    }
    if (!g_main_pwm_status.initialized || outputs == NULL || count == 0U || count > 4U) {
        ++g_main_pwm_status.rejected_frames;
        return false;
    }
    for (i = 0U; i < count; ++i) {
        uint8_t channel = outputs[i].main_channel;
        uint8_t bit;
        if (channel < 2U || channel > 5U || outputs[i].pulse_us < MAIN_PWM_MIN_US ||
            outputs[i].pulse_us > MAIN_PWM_MAX_US) {
            ++g_main_pwm_status.rejected_frames;
            return false;
        }
        bit = (uint8_t)(1U << (channel - 1U));
        if ((mask & bit) != 0U) {
            ++g_main_pwm_status.rejected_frames;
            return false;
        }
        mask |= bit;
        pulse[channel - 2U] = outputs[i].pulse_us;
    }
    MainPwmPort_Apply(mask, pulse);
    g_main_pwm_status.active_mask = mask;
    for (i = 0U; i < 4U; ++i) { g_main_pwm_status.pulse_us[i] = pulse[i]; }
    g_main_pwm_status.last_update_ms = now_ms;
    g_main_pwm_status.stop_reason = MAIN_PWM_STOP_NONE;
    ++g_main_pwm_status.accepted_frames;
    return true;
}

void MainPwm_Stop(void)
{
    stop(MAIN_PWM_STOP_EXPLICIT);
}

void MainPwm_Service(uint32_t now_ms)
{
    if (g_main_pwm_status.active_mask != 0U &&
        (uint32_t)(now_ms - g_main_pwm_status.last_update_ms) >= MAIN_PWM_TIMEOUT_MS) {
        stop(MAIN_PWM_STOP_TIMEOUT);
    }
}

void MainPwm_GetStatus(main_pwm_status_t *out)
{
    if (out != NULL) {
        *out = g_main_pwm_status;
        if (out->initialized) { MainPwmPort_GetState(&out->active_mask, out->pulse_us); }
    }
}

