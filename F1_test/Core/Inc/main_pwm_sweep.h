#ifndef MAIN_PWM_SWEEP_H
#define MAIN_PWM_SWEEP_H
#include "main_pwm.h"
#ifndef F1_TEST_WATCH_MODE
#define F1_TEST_WATCH_MODE 0
#endif
#if !F1_TEST_WATCH_MODE
typedef struct {
    uint32_t magic; /* 0x53575031 = SWP1 */
    uint32_t enabled; /* 1 starts automatically; set 0 to stop from Watch. */
    uint32_t duty_percent;
    uint32_t pulse_us;
    uint32_t transitions;
    uint32_t last_step_ms;
} main_pwm_sweep_status_t;
extern volatile main_pwm_sweep_status_t g_main_pwm_sweep;
/* Scope-only: MAIN2..5, 400 Hz, 10..90% in 10% steps every second, repeating.
 * Deliberately outside ESC pulse limits. No ESC/motor may be connected.
 * Call Init only AFTER MainPwm_Init succeeds. No shared actuator service. */
void MainPwmSweep_Init(uint32_t now_ms);
void MainPwmSweep_Service(uint32_t now_ms);
#endif
#endif
