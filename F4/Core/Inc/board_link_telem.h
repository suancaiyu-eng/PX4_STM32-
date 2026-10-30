#ifndef BOARD_LINK_TELEM_H
#define BOARD_LINK_TELEM_H

#include "board_link_protocol.h"

/* Used when IMU_TELEMETRY_VIEW_RC is selected (gyro is the default view).
 * SeekFree Assistant Printf: rc:value1,...,valueN\n, at most 8 fields.
 * Select 1..8 field IDs, in display order: 1..10 = raw S.BUS CH1..CH10,
 * 11 = valid, 12 = status, 13 = F1 age_ms. Rebuild after changing the list.
 * Default: CH1..CH8. Do not alternate groups: descriptions are not channel IDs.
 * Raw values may be retained on invalid input; select field 11/12 to inspect
 * validity, and check receive timeout if the board link stops delivering data.
 * IMU_TELEMETRY_RC_ONLY=0 restores the legacy #LINK_RX console. */
#ifndef BOARD_LINK_TELEM_RC_FIELDS
#define BOARD_LINK_TELEM_RC_FIELDS 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U
#endif

void BoardLinkTelem_Init(void);
/* For a shared foreground observer that also dispatches actuator responses. */
void BoardLinkTelem_OnMessage(const bl_message_t *message);
size_t BoardLinkTelem_Format(const bl_message_t *message, char *out, size_t capacity);

typedef struct {
    uint32_t received;
    uint32_t queued;
    uint32_t dropped;
} board_link_telem_status_t;
extern volatile board_link_telem_status_t g_board_link_telem_status;

#endif
