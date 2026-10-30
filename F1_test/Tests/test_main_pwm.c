#include "main_pwm.h"
#if F1_TEST_WATCH_MODE
#include "main_pwm_bench.h"
static bool bench_init(uint32_t now) { bool ok = MainPwm_Init(now); MainPwmBench_Init(); return ok; }
#define MainPwm_Init bench_init
#endif
#include "main_pwm_port.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>

static bool init_ok = true;
static unsigned applies, stops;
static uint8_t last_mask;
static uint16_t last_pulse[4];
bool MainPwmPort_Init(void) { return init_ok; }
void MainPwmPort_Stop(void) { ++stops; last_mask = 0U; }
void MainPwmPort_GetState(uint8_t *mask, uint16_t pulse[4])
{
    unsigned i;
    *mask = last_mask;
    for (i = 0; i < 4; ++i) { pulse[i] = (last_mask & (1U << (i + 1))) ? last_pulse[i] : 0; }
}
void MainPwmPort_Apply(uint8_t mask, const uint16_t pulse[4])
{
    unsigned i;
    ++applies;
    last_mask = mask;
    for (i = 0; i < 4; ++i) { last_pulse[i] = pulse[i]; }
}

int main(void)
{
    main_pwm_output_t outputs[4] = {{2,1000}, {3,1500}, {4,2000}, {5,1100}};
    main_pwm_status_t snapshot;
    unsigned i, previous;
    assert(MainPwm_Init(0));
    assert(stops && last_mask == 0 && g_main_pwm_status.active_mask == 0);
    for (i = 0; i < 4; ++i) {
        assert(MainPwm_Submit(&outputs[i], 1, i));
        assert(last_mask == (1U << (outputs[i].main_channel - 1)));
        assert(last_pulse[i] == outputs[i].pulse_us);
    }
    assert(MainPwm_Submit(outputs, 4, 10));
    assert(last_mask == 0x1e && last_pulse[1] == 1500 && last_pulse[2] == 2000);
    previous = applies;
    for (i = 0; i < 9; ++i) {
        main_pwm_output_t invalid = {(uint8_t)i, 1500};
        if (i >= 2 && i <= 5) { continue; }
        assert(!MainPwm_Submit(&invalid, 1, 100));
    }
    outputs[3].pulse_us = 999;
    assert(!MainPwm_Submit(outputs, 4, 100));
    outputs[3].pulse_us = 2001;
    assert(!MainPwm_Submit(outputs, 4, 100));
    outputs[3].pulse_us = 1500;
    outputs[3].main_channel = 3;
    assert(!MainPwm_Submit(outputs, 4, 100));
    outputs[3].main_channel = 5;
    assert(!MainPwm_Submit(NULL, 1, 100));
    assert(!MainPwm_Submit(outputs, 0, 100));
    assert(!MainPwm_Submit(outputs, 5, 100));
    assert(applies == previous && last_mask == 0x1e && g_main_pwm_status.last_update_ms == 10);
    MainPwm_Service(1009);
    assert(last_mask == 0x1e);
    MainPwm_Service(1010);
    assert(last_mask == 0 && g_main_pwm_status.stop_reason == MAIN_PWM_STOP_TIMEOUT);
    MainPwm_Service(1011);
    assert(last_mask == 0);
    assert(MainPwm_Submit(outputs, 4, 2000));
    assert(!MainPwm_Submit(outputs, 4, 3000)); /* Late submit stops/rejects. */
    assert(last_mask == 0);
    assert(MainPwm_Submit(outputs, 4, 3001)); /* Explicit new attempt. */
    MainPwm_Stop();
    assert(last_mask == 0);
    assert(MainPwm_Init(0xfffffff0U));
    assert(MainPwm_Submit(outputs, 4, 0xfffffff0U));
    MainPwm_Service(983U);
    assert(last_mask != 0);
    MainPwm_Service(984U);
    assert(last_mask == 0); /* Wraparound deadline. */
    init_ok = false;
    assert(!MainPwm_Init(0));
    assert(!MainPwm_Submit(outputs, 4, 0));
    assert(last_mask == 0 && g_main_pwm_status.stop_reason == MAIN_PWM_STOP_INIT_ERROR);
    init_ok = true;
    assert(MainPwm_Init(0));
    MainPwm_GetStatus(&snapshot);
    assert(snapshot.initialized && snapshot.active_mask == 0);
    MainPwm_GetStatus(NULL);
#if F1_TEST_WATCH_MODE
    assert(g_main_pwm_test.pulse_us == 1000 && g_main_pwm_test.duration_ms == 1000);
    MainPwmBench_Service(0);
    assert(last_mask == 0);
    g_main_pwm_test.command = 1;
    g_main_pwm_test.main_channel = 3;
    g_main_pwm_test.pulse_us = 2000;
    g_main_pwm_test.duration_ms = 10;
    g_main_pwm_test.sequence = 1;
    MainPwmBench_Service(100);
    assert(last_mask == 4 && g_main_pwm_test.result == MAIN_PWM_TEST_RUNNING);
    previous = applies;
    g_main_pwm_test.main_channel = 5; /* No new sequence: do not run/renew. */
    MainPwmBench_Service(109);
    assert(applies == previous && last_mask == 4);
    MainPwmBench_Service(110);
    assert(last_mask == 0 && g_main_pwm_test.result == MAIN_PWM_TEST_DONE);
    MainPwmBench_Service(111);
    assert(last_mask == 0 && applies == previous);
    g_main_pwm_test.sequence = 0; /* Replayed older mailbox. */
    MainPwmBench_Service(112);
    assert(applies == previous && g_main_pwm_test.acknowledged_sequence == 1);
    g_main_pwm_test.sequence = 2;
    MainPwmBench_Service(120);
    assert(last_mask == 0x10);
    g_main_pwm_test.command = 2;
    g_main_pwm_test.sequence = 3;
    MainPwmBench_Service(121);
    assert(last_mask == 0 && g_main_pwm_test.result == MAIN_PWM_TEST_STOPPED);
    g_main_pwm_test.command = 1;
    g_main_pwm_test.duration_ms = 1001;
    g_main_pwm_test.sequence = 4;
    MainPwmBench_Service(122);
    assert(last_mask == 0 && g_main_pwm_test.result == MAIN_PWM_TEST_INVALID);
    g_main_pwm_test.duration_ms = 0;
    g_main_pwm_test.sequence = 5;
    MainPwmBench_Service(123);
    assert(last_mask == 0 && g_main_pwm_test.result == MAIN_PWM_TEST_INVALID);
    g_main_pwm_test.duration_ms = 1000;
    g_main_pwm_test.main_channel = 258; /* No uint8 truncation acceptance. */
    g_main_pwm_test.sequence = 6;
    MainPwmBench_Service(124);
    assert(last_mask == 0 && g_main_pwm_test.result == MAIN_PWM_TEST_INVALID);
    g_main_pwm_test.main_channel = 2;
    g_main_pwm_test.sequence = 7;
    MainPwmBench_Service(0xfffffff0U);
    MainPwmBench_Service(983);
    assert(last_mask == 2);
    MainPwmBench_Service(984);
    MainPwm_Service(984);
    assert(last_mask == 0 && g_main_pwm_test.result == MAIN_PWM_TEST_DONE);
    /* Continuous mode lasts beyond 1 second, captures parameters once, and
     * cannot silently restart after an expired lease or explicit stop. */
    assert(MainPwm_Init(0));
    g_main_pwm_test.command = 3;
    g_main_pwm_test.main_channel = 2;
    g_main_pwm_test.duration_ms = 0;
    g_main_pwm_test.sequence = 1;
    MainPwmBench_Service(0);
    assert(last_mask == 2 && last_pulse[0] == 1000);
    g_main_pwm_test.main_channel = 5;
    g_main_pwm_test.pulse_us = 2000;
    for (i = 1; i <= 50; ++i) {
        MainPwmBench_Service(i * 100);
        MainPwm_Service(i * 100);
        assert(last_mask == 2 && last_pulse[0] == 1000);
        assert(g_main_pwm_test.result == MAIN_PWM_TEST_RUNNING);
    }
    MainPwmBench_Service(6000); /* Full second of missed service: no renewal. */
    assert(last_mask == 0 && g_main_pwm_test.result == MAIN_PWM_TEST_DRIVER_ERROR);
    previous = applies;
    MainPwmBench_Service(6001);
    assert(last_mask == 0 && applies == previous);
    g_main_pwm_test.sequence = 2;
    MainPwmBench_Service(6002);
    assert(last_mask == 0x10 && last_pulse[3] == 2000);
    g_main_pwm_test.command = 2;
    g_main_pwm_test.sequence = 3;
    MainPwmBench_Service(6003);
    MainPwmBench_Service(7003);
    assert(last_mask == 0 && g_main_pwm_test.result == MAIN_PWM_TEST_STOPPED);
    assert(MainPwm_Init(0xfffffff0U));
    g_main_pwm_test.command = 3; g_main_pwm_test.sequence = 1;
    MainPwmBench_Service(0xfffffff0U);
    MainPwmBench_Service(84);
    MainPwm_Service(84);
    assert(last_mask == 2 && g_main_pwm_test.result == MAIN_PWM_TEST_RUNNING);
    MainPwm_Stop(); MainPwmBench_Service(184);
    assert(last_mask == 0 && g_main_pwm_test.result == MAIN_PWM_TEST_DRIVER_ERROR);
    assert(MainPwm_Init(200)); MainPwmBench_Service(201);
    assert(last_mask == 0 && g_main_pwm_test.command == 0);
#else
    assert(last_mask == 0);
#endif
    puts("MAIN PWM policy / mailbox: PASS");
    return 0;
}
