#ifndef MAIN_PWM_H
#define MAIN_PWM_H
#include <stdbool.h>
#include <stdint.h>

#define MAIN_PWM_PERIOD_US 2500U
#define MAIN_PWM_MIN_US 1000U
#define MAIN_PWM_MAX_US 2000U
#define MAIN_PWM_TIMEOUT_MS 1000U
/* Mask bits use physical MAIN numbers: MAIN2 = bit 1, ... MAIN5 = bit 4. */
#define MAIN_PWM_MASK 0x1EU

typedef struct {
    uint8_t main_channel;
    uint16_t pulse_us;
} main_pwm_output_t;
typedef enum {
    MAIN_PWM_STOP_NONE, MAIN_PWM_STOP_EXPLICIT, MAIN_PWM_STOP_TIMEOUT,
    MAIN_PWM_STOP_INIT_ERROR = 4
} main_pwm_stop_reason_t;
typedef struct {
    bool initialized;
    uint8_t active_mask;
    uint16_t pulse_us[4]; /* MAIN2, MAIN3, MAIN4, MAIN5; zero when disabled. */
    uint32_t last_update_ms;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    main_pwm_stop_reason_t stop_reason;
} main_pwm_status_t;
extern volatile main_pwm_status_t g_main_pwm_status;

/* Main-loop only. Submit replaces the whole frame; omitted channels go low.
 * First submit explicitly enables output. Valid repeated submits renew timeout.
 * count=0 / NULL / duplicate channels / invalid values are rejected atomically.
 * After expiry Submit stops/rejects once; a NEW explicit Submit may restart. */
bool MainPwm_Init(uint32_t now_ms);
bool MainPwm_Submit(const main_pwm_output_t *outputs, uint8_t count, uint32_t now_ms);
void MainPwm_Stop(void);
void MainPwm_Service(uint32_t now_ms);
void MainPwm_GetStatus(main_pwm_status_t *out);

#endif
