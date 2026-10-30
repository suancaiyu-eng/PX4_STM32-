#ifndef ACTUATOR_IO_H
#define ACTUATOR_IO_H
#include "actuator_protocol.h"
#include "board_link_app.h"
extern volatile act_status_t g_actuator_io_status;
extern volatile uint32_t g_actuator_io_tx_drops;
void ActuatorIo_Init(uint32_t boot_id, bool remote_allowed);
void ActuatorIo_Service(void);
bool ActuatorIo_HandleRequest(const bl_message_t *request);
#endif
