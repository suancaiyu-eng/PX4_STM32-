#include "main_pwm_sweep.h"
#include "main_pwm_port.h"
#include <assert.h>
#include <stdio.h>

static unsigned applies, stops;
static uint8_t mask;
static uint16_t width;
void MainPwmPort_Apply(uint8_t m, const uint16_t pulse[4])
{
    assert(m == 0x1e && pulse[1] == pulse[0] && pulse[2] == pulse[0] && pulse[3] == pulse[0]);
    mask = m; width = pulse[0]; ++applies;
}
void MainPwmPort_Stop(void) { mask = 0; ++stops; }
int main(void)
{
    unsigned i;
    MainPwmSweep_Init(100);
    assert(mask == 0x1e && width == 250 && applies == 1);
    assert(g_main_pwm_sweep.magic == 0x53575031U && g_main_pwm_sweep.duty_percent == 10);
    MainPwmSweep_Service(1099); assert(applies == 1);
    for (i = 1; i <= 18; ++i) {
        unsigned duty = ((i % 9) + 1) * 10;
        MainPwmSweep_Service(100 + i * 1000);
        assert(g_main_pwm_sweep.duty_percent == duty && width == duty * 25);
        assert(g_main_pwm_sweep.transitions == i && mask == 0x1e);
    }
    g_main_pwm_sweep.enabled = 0;
    MainPwmSweep_Service(18101); assert(mask == 0 && stops == 1);
    MainPwmSweep_Service(19101); assert(stops == 1);
    g_main_pwm_sweep.enabled = 1;
    MainPwmSweep_Service(19102); assert(mask == 0x1e && width == 250);
    MainPwmSweep_Init(0xfffffff0U);
    MainPwmSweep_Service(983); assert(width == 250);
    MainPwmSweep_Service(984); assert(width == 500);
    MainPwmSweep_Service(10000); assert(width == 750); /* One visible step, no burst catch-up. */
    puts("PASS: MAIN2..5 auto 10..90% sweep, 1s hold, repeat, stop/resume and tick wrap");
    return 0;
}
