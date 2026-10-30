#ifndef ACTUATOR_CLIENT_H
#define ACTUATOR_CLIENT_H
#include "actuator_protocol.h"
#include "board_link_app.h"
#ifndef ACTUATOR_CLIENT_TEST_ENABLED
#define ACTUATOR_CLIENT_TEST_ENABLED 0
#endif
typedef struct {
    bool ready, pending, status_valid;
    uint32_t accepted_responses, rejected_responses, response_timeouts, tx_full;
    uint32_t last_response_ms;
    act_status_t io;
} actuator_client_status_t;
extern volatile actuator_client_status_t g_actuator_client;
/* Main-loop APIs. No automatic OPEN, enable, retransmit or output refresh.
 * One outstanding request. Call OnMessage from the board-link observer. */
void ActuatorClient_Init(void);
bool ActuatorClient_Open(void);
bool ActuatorClient_Set(bool enable, uint8_t mask, const uint16_t pulse_us[4], uint16_t ttl_ms);
bool ActuatorClient_Disable(void); /* Can supersede a pending SET. */
bool ActuatorClient_Query(void);
void ActuatorClient_OnMessage(const bl_message_t *message);
void ActuatorClient_Service(void);
#if ACTUATOR_CLIENT_TEST_ENABLED
typedef struct {
    uint32_t command; /* 1=OPEN then SET once; 2=disable/cancel; 3=query. */
    uint32_t mask, pulse_us[4], ttl_ms;
    uint32_t sequence, acknowledged_sequence;
    uint32_t result; /* 0 idle,1 opening,2 sent,3 done,4 rejected,5 stopped. */
} actuator_client_test_t;
extern volatile actuator_client_test_t g_actuator_client_test;
#endif
#endif
