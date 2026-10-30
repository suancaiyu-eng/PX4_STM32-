#ifndef MAIN_PWM_PORT_H
#define MAIN_PWM_PORT_H
#include <stdbool.h>
#include <stdint.h>
/* STM32 adapter; only main_pwm.c owns these functions. */
bool MainPwmPort_Init(void);
void MainPwmPort_Apply(uint8_t mask, const uint16_t pulse_us[4]);
void MainPwmPort_Stop(void);
/* Read timer enable, pin routing and configured compare (preload) registers.
 * This is configuration readback, NOT measurement of the physical waveform. */
void MainPwmPort_GetState(uint8_t *mask, uint16_t pulse_us[4]);
#endif
