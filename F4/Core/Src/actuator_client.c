#include "actuator_client.h"
#include <stddef.h>
#include <string.h>

volatile actuator_client_status_t g_actuator_client;
static uint32_t client_nonce, command_sequence, pending_app_sequence;
static uint32_t pending_since, anchor_local_ms, anchor_io_ms;
static uint32_t session_boot, session_id;
static uint16_t anchor_remaining_ms;
static uint16_t pending_command, pending_transport_sequence;
static bool pending_enable;
#if ACTUATOR_CLIENT_TEST_ENABLED
volatile actuator_client_test_t g_actuator_client_test;
static uint32_t test_sequence;
static bool test_opening, test_sent;
static uint8_t test_mask;
static uint16_t test_pulse[4], test_ttl;
#endif

static void invalidate(void)
{
    g_actuator_client.ready = false;
    g_actuator_client.pending = false;
    session_boot = session_id = 0U;
}
void ActuatorClient_Init(void)
{
    memset((void *)&g_actuator_client, 0, sizeof(g_actuator_client));
    client_nonce = command_sequence = 0U;
    session_boot = session_id = 0U; anchor_remaining_ms = 0U;
#if ACTUATOR_CLIENT_TEST_ENABLED
    memset((void *)&g_actuator_client_test, 0, sizeof(g_actuator_client_test));
    g_actuator_client_test.mask = 0x02U;
    g_actuator_client_test.pulse_us[0] = 1000U;
    g_actuator_client_test.ttl_ms = ACT_TTL_DEFAULT_MS;
    test_sequence = 0U; test_opening = test_sent = false;
#endif
}
static bool request(uint16_t command, const void *payload, uint16_t size)
{
    if (!BoardLink_SendRequest(command, payload, size, &pending_transport_sequence)) {
        ++g_actuator_client.tx_full;
        return false;
    }
    pending_command = command;
    pending_since = HAL_GetTick();
    g_actuator_client.pending = true;
    return true;
}
bool ActuatorClient_Open(void)
{
    uint8_t payload[ACT_OPEN_SIZE];
    if (g_actuator_client.pending || client_nonce == 0xFFFFFFFFUL) { return false; }
    invalidate();
    Act_EncodeOpen(payload, ++client_nonce);
    return request(ACT_CMD_OPEN, payload, sizeof(payload));
}
bool ActuatorClient_Set(bool enable, uint8_t mask, const uint16_t pulse_us[4], uint16_t ttl_ms)
{
    act_set_t set;
    uint8_t payload[ACT_SET_SIZE], i;
    uint32_t now = HAL_GetTick();
    if (!g_actuator_client.ready || g_actuator_client.pending || pulse_us == NULL) { return false; }
    /* Never issue from a locally expired status/clock anchor. */
    if ((uint32_t)(now - anchor_local_ms) >= anchor_remaining_ms) {
        invalidate(); return false;
    }
    memset(&set, 0, sizeof(set));
    set.boot = session_boot; set.session = session_id;
    set.sequence = command_sequence + 1U;
    set.issued_ms = anchor_io_ms + (uint32_t)(now - anchor_local_ms);
    set.enable = enable ? 1U : 0U; set.mask = mask; set.ttl_ms = ttl_ms;
    for (i = 0U; i < 4U; ++i) { set.pulse_us[i] = pulse_us[i]; }
    if (Act_ValidateSet(&set) != ACT_OK) { return false; }
    Act_EncodeSet(payload, &set);
    if (!request(ACT_CMD_SET, payload, sizeof(payload))) { return false; }
    command_sequence = set.sequence;
    pending_app_sequence = set.sequence;
    pending_enable = enable;
    return true;
}
bool ActuatorClient_Disable(void)
{
    uint16_t zero[4] = {0U, 0U, 0U, 0U};
    /* Cancel local test/open continuation before touching transport state. */
#if ACTUATOR_CLIENT_TEST_ENABLED
    test_opening = test_sent = false;
#endif
    if (g_actuator_client.pending && pending_command == ACT_CMD_OPEN) {
        invalidate(); return true; /* OPEN alone cannot enable any output. */
    }
    if (g_actuator_client.pending) {
        g_actuator_client.pending = false; /* Higher app sequence supersedes it. */
    }
    return ActuatorClient_Set(false, 0U, zero, ACT_TTL_DEFAULT_MS);
}
bool ActuatorClient_Query(void)
{
    if (g_actuator_client.pending) { return false; }
    return request(ACT_CMD_STATUS, NULL, 0U);
}
void ActuatorClient_OnMessage(const bl_message_t *m)
{
    act_status_t s;
    uint32_t now = HAL_GetTick();
    bool matches;
    if (m == NULL || m->source != BL_NODE_F1 || m->destination != BL_NODE_F4 ||
        (m->command != ACT_CMD_OPEN && m->command != ACT_CMD_SET && m->command != ACT_CMD_STATUS)) { return; }
    if (!Act_DecodeStatus(m->payload, m->length, &s)) {
        ++g_actuator_client.rejected_responses; return;
    }
    if (m->kind == BL_KIND_EVENT && m->command == ACT_CMD_STATUS) {
        /* Events can revoke permission, never grant or renew it. */
        if (g_actuator_client.ready && (s.boot != session_boot || s.session != session_id ||
            (s.state != ACT_STATE_READY && s.state != ACT_STATE_ACTIVE))) {
            invalidate();
        }
        /* Observe even when closed. Permission/clock lease stays private and
         * is never refreshed by this diagnostic snapshot. */
        g_actuator_client.io = s;
        g_actuator_client.status_valid = true;
        g_actuator_client.last_response_ms = now;
        return;
    }
    matches = m->kind == BL_KIND_RESPONSE && g_actuator_client.pending &&
              m->command == pending_command && m->sequence == pending_transport_sequence;
    if (!matches) { ++g_actuator_client.rejected_responses; return; }
    if ((uint32_t)(now - pending_since) > ACT_MAX_SYNC_RTT_MS) {
        ++g_actuator_client.rejected_responses; invalidate(); return;
    }
    if ((pending_command == ACT_CMD_OPEN && s.client_nonce != client_nonce && s.result == ACT_OK) ||
        (pending_command == ACT_CMD_SET && s.request_sequence != pending_app_sequence)) {
        ++g_actuator_client.rejected_responses; invalidate(); return;
    }
    if (pending_command == ACT_CMD_SET && s.result == ACT_OK &&
        (s.boot != session_boot || (pending_enable &&
        (s.session != session_id || s.state != ACT_STATE_ACTIVE)) ||
        (!pending_enable && (s.state != ACT_STATE_CLOSED || s.mask != 0U)) ||
        s.last_sequence != pending_app_sequence)) {
        ++g_actuator_client.rejected_responses; invalidate(); return;
    }
    if (pending_command == ACT_CMD_STATUS && g_actuator_client.ready &&
        (s.boot != session_boot || s.session != session_id)) {
        invalidate();
    }
    ++g_actuator_client.accepted_responses;
    g_actuator_client.pending = false;
    g_actuator_client.status_valid = true;
    g_actuator_client.last_response_ms = now;
    g_actuator_client.io = s;
    if (pending_command == ACT_CMD_OPEN) {
        g_actuator_client.ready = s.result == ACT_OK && s.state == ACT_STATE_READY && s.flags == 3U;
        command_sequence = 0U;
        if (g_actuator_client.ready) { session_boot = s.boot; session_id = s.session; }
    } else if (s.result != ACT_OK || s.state == ACT_STATE_CLOSED || s.state == ACT_STATE_UNAVAILABLE) {
        g_actuator_client.ready = false;
    } else if (g_actuator_client.ready && (s.boot == 0U || s.session == 0U)) {
        g_actuator_client.ready = false;
    }
    /* A conservative clock anchor: F1 tick when reply was queued, local tick
     * when received. Downlink delay subtracts from validity, never extends it. */
    anchor_io_ms = s.io_ms;
    anchor_local_ms = now;
    anchor_remaining_ms = s.remaining_ms;
}

#if ACTUATOR_CLIENT_TEST_ENABLED
static void test_service(void)
{
    uint32_t seq = g_actuator_client_test.sequence;
    if (seq != test_sequence && (uint32_t)(seq - test_sequence) < 0x80000000UL) {
        uint32_t command = g_actuator_client_test.command;
        uint32_t mask = g_actuator_client_test.mask, ttl = g_actuator_client_test.ttl_ms;
        uint32_t values[4];
        uint8_t i;
        bool valid = mask <= 0xFFU && ttl <= 0xFFFFU;
        for (i = 0U; i < 4U; ++i) {
            values[i] = g_actuator_client_test.pulse_us[i];
            if (values[i] > 0xFFFFU) { valid = false; }
        }
        test_sequence = seq;
        if (command == 2U) {
            test_opening = test_sent = false;
            g_actuator_client_test.result = ActuatorClient_Disable() ? 5U : 4U;
        } else if (command == 3U) {
            g_actuator_client_test.result = ActuatorClient_Query() ? 2U : 4U;
        } else if (command == 1U && valid && !test_opening && !test_sent) {
            act_set_t set;
            memset(&set, 0, sizeof(set));
            set.enable = 1U; set.mask = (uint8_t)mask; set.ttl_ms = (uint16_t)ttl;
            for (i = 0U; i < 4U; ++i) { set.pulse_us[i] = (uint16_t)values[i]; }
            if (Act_ValidateSet(&set) == ACT_OK && ActuatorClient_Open()) {
                test_mask = set.mask; test_ttl = set.ttl_ms;
                memcpy(test_pulse, set.pulse_us, sizeof(test_pulse));
                test_opening = true;
                g_actuator_client_test.result = 1U;
            } else { g_actuator_client_test.result = 4U; }
        } else { g_actuator_client_test.result = 4U; }
        g_actuator_client_test.acknowledged_sequence = seq;
    }
    if (test_opening && !g_actuator_client.pending) {
        test_opening = false;
        if (ActuatorClient_Set(true, test_mask, test_pulse, test_ttl)) {
            test_sent = true; g_actuator_client_test.result = 2U;
        } else { g_actuator_client_test.result = 4U; }
    }
    if (test_sent && !g_actuator_client.pending && !g_actuator_client.ready) {
        test_sent = false;
        g_actuator_client_test.result = 3U;
    }
}
#endif
void ActuatorClient_Service(void)
{
    uint32_t now = HAL_GetTick();
    if (g_actuator_client.pending && (uint32_t)(now - pending_since) >= ACT_REPLY_TIMEOUT_MS) {
        ++g_actuator_client.response_timeouts;
        invalidate();
    }
    if (g_actuator_client.ready && !g_actuator_client.pending &&
        (uint32_t)(now - anchor_local_ms) >= anchor_remaining_ms) { invalidate(); }
#if ACTUATOR_CLIENT_TEST_ENABLED
    test_service();
#endif
}
