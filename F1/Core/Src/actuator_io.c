#include "actuator_io.h"
#include "main_pwm.h"
#include <string.h>

volatile act_status_t g_actuator_io_status;
volatile uint32_t g_actuator_io_tx_drops;
static uint32_t generation, lease_start, lease_duration, last_event_ms;
static bool allowed, event_pending;

static void close_session(uint8_t reason)
{
    if (allowed) { MainPwm_Stop(); }
    g_actuator_io_status.state = allowed ? ACT_STATE_CLOSED : ACT_STATE_UNAVAILABLE;
    g_actuator_io_status.stop_reason = reason;
    g_actuator_io_status.session = 0U;
    event_pending = true;
}

static void refresh(uint32_t now)
{
    main_pwm_status_t pwm;
    uint8_t i;
    bool mismatch = false;
    MainPwm_GetStatus(&pwm);
    for (i = 0U; i < 4U; ++i) {
        if (pwm.pulse_us[i] != g_main_pwm_status.pulse_us[i]) { mismatch = true; }
    }
    if ((g_actuator_io_status.state == ACT_STATE_READY || g_actuator_io_status.state == ACT_STATE_ACTIVE) &&
        (uint32_t)(now - lease_start) >= lease_duration) { close_session(ACT_STOP_EXPIRED); }
    if (g_actuator_io_status.state == ACT_STATE_ACTIVE &&
        (!pwm.initialized || mismatch || pwm.active_mask != g_main_pwm_status.active_mask || pwm.active_mask == 0U)) {
        close_session(ACT_STOP_PWM);
        g_actuator_io_status.result = ACT_PWM_ERROR;
    }
    /* Refresh after Stop: never report a cached command as the active output. */
    MainPwm_GetStatus(&pwm);
    g_actuator_io_status.io_ms = now;
    g_actuator_io_status.flags = (uint8_t)((pwm.initialized ? 1U : 0U) | (allowed ? 2U : 0U));
    g_actuator_io_status.mask = pwm.active_mask;
    for (i = 0U; i < 4U; ++i) { g_actuator_io_status.pulse_us[i] = pwm.pulse_us[i]; }
    g_actuator_io_status.remaining_ms =
        g_actuator_io_status.state == ACT_STATE_READY || g_actuator_io_status.state == ACT_STATE_ACTIVE ?
        (uint16_t)(lease_duration - (uint32_t)(now - lease_start)) : 0U;
}

void ActuatorIo_Init(uint32_t boot_id, bool remote_allowed)
{
    memset((void *)&g_actuator_io_status, 0, sizeof(g_actuator_io_status));
    g_actuator_io_tx_drops = 0U;
    allowed = remote_allowed && boot_id != 0U && g_main_pwm_status.initialized;
    generation = 0U; last_event_ms = HAL_GetTick() - 5U;
    g_actuator_io_status.boot = boot_id;
    close_session(ACT_STOP_BOOT);
    refresh(HAL_GetTick());
    BoardLinkApp_SetCommandHandler(ActuatorIo_HandleRequest);
}

static void send_status(uint8_t kind, uint16_t command, uint16_t sequence)
{
    act_status_t snapshot = g_actuator_io_status;
    uint8_t payload[ACT_STATUS_SIZE];
    Act_EncodeStatus(payload, &snapshot);
    if (!BoardLink_Send(kind, command, sequence, payload, sizeof(payload))) {
        ++g_actuator_io_tx_drops;
        event_pending = true;
    }
}

bool ActuatorIo_HandleRequest(const bl_message_t *m)
{
    act_result_t result = ACT_OK;
    uint32_t now = HAL_GetTick();
    if (m == NULL || (m->command != ACT_CMD_OPEN && m->command != ACT_CMD_SET &&
        m->command != ACT_CMD_STATUS)) { return false; }
    refresh(now); /* Check expiry BEFORE queued commands, including OPEN/SET. */
    if (m->source != BL_NODE_F4 || m->destination != BL_NODE_F1 || m->kind != BL_KIND_REQUEST) {
        return true;
    }
    g_actuator_io_status.request_sequence = 0U;
    if (m->command == ACT_CMD_STATUS) {
        if (m->length != 0U) { result = ACT_BAD_LENGTH; }
    } else if (m->command == ACT_CMD_OPEN) {
        if (m->length != ACT_OPEN_SIZE) { result = ACT_BAD_LENGTH; }
        else if (m->payload[0] != ACT_VERSION || m->payload[1] != 0U || m->payload[2] != 0U ||
                 m->payload[3] != 0U || Act_Get32(m->payload + 4U) == 0U) { result = ACT_BAD_FORMAT; }
        else if (!allowed || generation == 0xFFFFFFFFUL) { result = ACT_UNAVAILABLE; }
        else {
            close_session(ACT_STOP_OPEN);
            g_actuator_io_status.client_nonce = Act_Get32(m->payload + 4U);
            g_actuator_io_status.session = ++generation;
            g_actuator_io_status.last_sequence = 0U;
            g_actuator_io_status.state = ACT_STATE_READY;
            lease_start = now; lease_duration = ACT_READY_TIMEOUT_MS;
        }
    } else {
        act_set_t set;
        result = Act_DecodeSet(m->payload, m->length, &set);
        if (m->length == ACT_SET_SIZE) { g_actuator_io_status.request_sequence = Act_Get32(m->payload + 12U); }
        if (result == ACT_OK) {
            uint32_t delta = set.sequence - g_actuator_io_status.last_sequence;
            if (!allowed) { result = ACT_UNAVAILABLE; }
            else if (set.boot != g_actuator_io_status.boot || set.session == 0U ||
                     set.session != g_actuator_io_status.session) { result = ACT_BAD_SESSION; }
            else if (delta == 0U || delta >= 0x80000000UL) { result = ACT_BAD_SEQUENCE; }
            else if ((uint32_t)(now - set.issued_ms) >= set.ttl_ms) { result = ACT_STALE; }
            else {
                main_pwm_output_t outputs[4];
                uint8_t i, count = 0U;
                if (!set.enable) { close_session(ACT_STOP_DISABLED); }
                else {
                    for (i = 0U; i < 4U; ++i) {
                        if ((set.mask & (1U << (i + 1U))) != 0U) {
                            outputs[count].main_channel = i + 2U;
                            outputs[count++].pulse_us = set.pulse_us[i];
                        }
                    }
                    if (!MainPwm_Submit(outputs, count, now)) {
                        close_session(ACT_STOP_PWM); result = ACT_PWM_ERROR;
                    } else {
                        g_actuator_io_status.state = ACT_STATE_ACTIVE;
                        g_actuator_io_status.stop_reason = ACT_STOP_NONE;
                        lease_start = set.issued_ms; lease_duration = set.ttl_ms;
                    }
                }
                if (result == ACT_OK) {
                    g_actuator_io_status.last_sequence = set.sequence;
                    ++g_actuator_io_status.accepted;
                }
            }
        }
    }
    if (result != ACT_OK) { ++g_actuator_io_status.rejected; }
    g_actuator_io_status.result = (uint8_t)result;
    refresh(now);
    send_status(BL_KIND_RESPONSE, m->command, m->sequence);
    return true;
}

void ActuatorIo_Service(void)
{
    uint32_t now = HAL_GetTick();
    refresh(now);
    if ((event_pending && (uint32_t)(now - last_event_ms) >= 5U) ||
        (uint32_t)(now - last_event_ms) >= 50U) {
        event_pending = false;
        send_status(BL_KIND_EVENT, ACT_CMD_STATUS, 0U);
        last_event_ms = now;
    }
}
