/*
(C) tblaha 2023
 */

#pragma once

#include <string.h>


void initPiTelemetry(void);
void handlePiTelemetry(void);
void checkPiTelemetryState(void);

void freePiTelemetryPort(void);
void configurePiTelemetryPort(void);


#if defined(USE_TELEMETRY_PI)

// --- Protocol definition ---
#define PI_STX 0xFE
#define PI_ESC 0x01
#define PI_STX_ESC 0x02
#define PI_ESC_ESC 0x03
#define PI_MAX_PAYLOAD_LEN 63
#define PI_ID_LEN 1
#define PI_MAX_PACKET_LEN (PI_MAX_PAYLOAD_LEN + PI_ID_LEN)
typedef struct __pi_message {
    uint8_t len; // not sent
    uint8_t id;
    uint32_t payload[(2*PI_MAX_PAYLOAD_LEN+3)/4];
} __attribute__((packed)) pi_message_t;

// --- utilities ---
#define PI_MSG_TO_SEND_BUFFER(_BUFFER, _MSG) \
    memcpy((_BUFFER), (const uint8_t *)( &((_MSG).id) ), (_MSG).len)


/*
void pi_msg_to_send_buffer(uint8_t *buffer, const pi_message_t *msg)
{
    memcpy(buffer, (const uint8_t *)( &(msg->start) ), msg->len);
}
*/

//--------------------------
// ------ MESSAGES ---------
//--------------------------


// ------ IMU ------
#define PI_MSG_IMU_ID 1

// payload definition
typedef struct __pi_IMU_t
{
    uint32_t time_ms;
    float roll;
    float pitch;
    float yaw;
    float x;
    float y;
    float z;
} __attribute__((packed)) pi_IMU_t;
#define PI_MSG_IMU_PAYLOAD_LEN (7*4)

// packing helper
static inline void pi_msg_IMU_pack(pi_message_t* msg, uint32_t time_ms, float roll, float pitch, float yaw, float x, float y, float z)
{
    pi_IMU_t payload;
    payload.time_ms = time_ms;
    payload.roll = roll;
    payload.pitch = pitch;
    payload.yaw = yaw;
    payload.x = x;
    payload.y = y;
    payload.z = z;

    memcpy((char *)( &(msg->payload[0]) ), &payload, PI_MSG_IMU_PAYLOAD_LEN);
    msg->id = PI_MSG_IMU_ID;
    msg->len = PI_MSG_IMU_PAYLOAD_LEN + PI_ID_LEN;
}

#endif