#ifndef PROTOCOL_DEF_H
#define PROTOCOL_DEF_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Frame Types ----
#define MSG_MISSION 0xA1
#define MSG_ACK     0x06
#define MSG_NACK    0x15
#define MSG_STATUS  0xA2
#define MSG_CMD     0xC0

// ---- Command Types carried by MSG_CMD ----
#define CMD_START   1
#define CMD_ABORT   2

// ---- MAVLink TUNNEL Parameters ----
#define TUNNEL_PAYLOAD_TYPE 38501
#define TUNNEL_PAYLOAD_SIZE 128
#define CHUNK_HEADER_SIZE   4
#define MAX_CHUNK_DATA      (TUNNEL_PAYLOAD_SIZE - CHUNK_HEADER_SIZE) // 124 bytes

#pragma pack(push, 1)

// Individual waypoint in the serialized mission blob
typedef struct {
    int32_t  lat_e7;   // Latitude degrees * 1e7
    int32_t  lon_e7;   // Longitude degrees * 1e7
    uint16_t alt_m;    // Relative altitude in meters
} WaypointWire;

// TUNNEL Chunk Header (first 4 bytes of each 128-byte TUNNEL payload)
typedef struct {
    uint8_t frame_type; // MSG_MISSION, MSG_ACK, MSG_NACK, MSG_STATUS, MSG_CMD
    uint8_t seq;        // 0-based chunk index
    uint8_t total;      // Total number of chunks in this message
    uint8_t data_len;   // Number of valid payload bytes following
} TunnelChunkHeader;

// ACK Payload structure carried within a MSG_ACK frame
typedef struct {
    uint8_t  waypoint_count;
    uint16_t crc16;
} AckPayload;

// Command Payload structure carried within a MSG_CMD frame
typedef struct {
    uint8_t command_id; // CMD_START or CMD_ABORT
} CmdPayload;

#pragma pack(pop)

// CRC16-CCITT calculation (Polynomial 0x1021, init 0xFFFF, no reflect, no final XOR)
static inline uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_DEF_H

