#include "board_link_telem.h"
#include "board_link_app.h"
#include "imu_telemetry.h"
#include "ms56xx_baro.h"
#include "rc_link.h"
#include <stdio.h>
#include <string.h>

volatile board_link_telem_status_t g_board_link_telem_status;

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) |
           ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
}

#if !IMU_TELEMETRY_RC_ONLY
size_t BoardLinkTelem_Format(const bl_message_t *message, char *out, size_t capacity)
{
    static const char hex_digit[] = "0123456789ABCDEF";
    char hex[BL_MAX_PAYLOAD * 2U + 1U];
    char ascii[BL_MAX_PAYLOAD + 1U];
    char test_fields[64];
    const char *kind;
    uint16_t i, text_start = 0U;
    int length;
    if (message == NULL || out == NULL || capacity == 0U || message->length > BL_MAX_PAYLOAD) {
        return 0U;
    }
    kind = message->kind == BL_KIND_REQUEST ? "REQ" :
           message->kind == BL_KIND_RESPONSE ? "RSP" :
           message->kind == BL_KIND_EVENT ? "EVENT" : "ERROR";
    test_fields[0] = '\0';
    if (message->command == BL_CMD_PING && message->length == 23U) {
        length = snprintf(test_fields, sizeof(test_fields), " count=%lu sender_ms=%lu",
            (unsigned long)get32(message->payload), (unsigned long)get32(message->payload + 4U));
        if (length < 0 || (size_t)length >= sizeof(test_fields)) { return 0U; }
        text_start = 8U;
    }
    for (i = 0U; i < message->length; ++i) {
        uint8_t byte = message->payload[i];
        hex[i * 2U] = hex_digit[byte >> 4U];
        hex[i * 2U + 1U] = hex_digit[byte & 15U];
        if (i >= text_start) {
            ascii[i - text_start] = byte >= 32U && byte <= 126U && byte != '"' && byte != '\\' ? (char)byte : '.';
        }
    }
    hex[message->length * 2U] = '\0';
    ascii[message->length - text_start] = '\0';
    length = snprintf(out, capacity,
        "#LINK_RX src=F%u type=%s cmd=0x%04X seq=%u len=%u%s text=\"%s\" hex=%s\n",
        (unsigned int)message->source, kind, (unsigned int)message->command,
        (unsigned int)message->sequence, (unsigned int)message->length, test_fields, ascii, hex);
    if (length <= 0 || (size_t)length >= capacity) { return 0U; }
    return (size_t)length;
}
#else
static const uint8_t rc_fields[] = { BOARD_LINK_TELEM_RC_FIELDS };
typedef char RcFieldCountCheck[(sizeof(rc_fields) >= 1U && sizeof(rc_fields) <= 8U) ? 1 : -1];

size_t BoardLinkTelem_Format(const bl_message_t *message, char *out, size_t capacity)
{
    const uint8_t *p;
    unsigned long values[RC_LINK_CHANNELS + 3U];
    unsigned int i;
    size_t used = 0U;
    int length;
    if (message == NULL || out == NULL || capacity == 0U ||
        message->source != BL_NODE_F1 || message->kind != BL_KIND_EVENT ||
        message->command != BL_CMD_RC_INPUT || message->length != RC_LINK_SIZE) { return 0U; }
    p = message->payload;
    if (p[0] != RC_LINK_VERSION || p[1] > 4U || p[3] != RC_LINK_CHANNELS ||
        (p[2] & 0xC0U) != 0U) { return 0U; }
    for (i = 0U; i < RC_LINK_CHANNELS; ++i) {
        values[i] = (unsigned int)p[40U + 2U*i] | ((unsigned int)p[41U + 2U*i] << 8U);
        if (values[i] > 2047U) { return 0U; }
    }
    values[10] = p[1] == 1U && (p[2] & 15U) == 3U && get32(p + 8U) < 100U;
    values[11] = p[1];
    values[12] = get32(p + 8U);
    /* Same fields on every sample, comma-separated, exactly one final LF.
     * Format in the foreground and keep the existing UART DMA owner. */
    for (i = 0U; i < sizeof(rc_fields); ++i) {
        if (rc_fields[i] < 1U || rc_fields[i] > RC_LINK_CHANNELS + 3U) { return 0U; }
        length = snprintf(out + used, capacity - used, "%s%lu%s",
            i == 0U ? "rc:" : ",", values[rc_fields[i] - 1U],
            i + 1U == sizeof(rc_fields) ? "\n" : "");
        if (length <= 0 || (size_t)length >= capacity - used) { return 0U; }
        used += (size_t)length;
    }
    return used;
}
#endif

void BoardLinkTelem_OnMessage(const bl_message_t *message)
{
#if BARO_TELEMETRY_ENABLED || (IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_SEEKFREE && IMU_TELEMETRY_VIEW != IMU_TELEMETRY_VIEW_RC)
    /* Dedicated barometer console: FireWater has only one numeric channel set. */
    (void)message;
#else
    char line[IMU_TELEMETRY_TX_BUFFER_SIZE];
    size_t length;
#if IMU_TELEMETRY_RC_ONLY
    /* PING requests/responses still run on the board link, without console output. */
    if (message == NULL || message->source != BL_NODE_F1 ||
        message->kind != BL_KIND_EVENT || message->command != BL_CMD_RC_INPUT) { return; }
#endif
    ++g_board_link_telem_status.received;
    length = BoardLinkTelem_Format(message, line, sizeof(line));
    if (length != 0U && ImuTelemetry_QueueText(line, (uint16_t)length)) {
        ++g_board_link_telem_status.queued;
    } else {
        ++g_board_link_telem_status.dropped;
    }
#endif
}

void BoardLinkTelem_Init(void)
{
    memset((void *)&g_board_link_telem_status, 0, sizeof(g_board_link_telem_status));
    BoardLinkApp_SetMessageHandler(BoardLinkTelem_OnMessage);
}
