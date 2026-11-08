"""Shared mission codec for the companion-drone link.

This module is the single source of truth for how a waypoint mission is turned
into bytes on the ground side and turned back into waypoints on the Pi side.
It is deliberately dependency-free (pure stdlib) so it can be unit-tested off
the vehicle and imported identically by the ground router and the Pi autonomy.

Wire format
-----------
A mission is first serialized into a compact *blob*:

    [0]       uint8   message type   (0xA1 = mission upload)
    [1]       uint8   waypoint count N (0..255)
    [2..]     N x {   int32  lat_e7   (degrees * 1e7, little-endian)
                      int32  lon_e7
                      uint16 alt_m    (relative altitude, metres) }
    [end-2..] uint16  CRC16-CCITT over everything before the CRC

That blob is then split into *chunks* small enough to ride inside a single
MAVLink TUNNEL message payload (128 bytes). Each chunk carries its own tiny
header so the receiver can reassemble out-of-order / lossy delivery:

    [0] uint8  frame type (0xA1 mission / 0x06 ACK / 0x15 NACK / 0xA2 status)
    [1] uint8  seq        (0-based chunk index)
    [2] uint8  total      (total number of chunks in this blob)
    [3] uint8  data_len   (number of valid data bytes that follow)
    [4..]      data_len bytes of blob payload

Max data per chunk = 128 - 4 = 124 bytes.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Dict, List, Optional

# ---- frame type ids ---------------------------------------------------------
MSG_MISSION = 0xA1
MSG_ACK = 0x06
MSG_NACK = 0x15
MSG_STATUS = 0xA2
MSG_CMD = 0xC0  # operator command (start / abort), data[0] = command id

# command ids carried by MSG_CMD
CMD_START = 1    # begin the mission (prearm -> takeoff -> fly)
CMD_ABORT = 2    # stop autonomy; hand back to GUIDED hover / pilot
CMD_TAKEOFF = 3  # takeoff at current location to specified altitude (data[1:5] = float alt)
CMD_LAND = 4     # land immediately at current position

# TUNNEL payload is a fixed 128-byte field; we reserve 4 for the chunk header.
TUNNEL_PAYLOAD_SIZE = 128
CHUNK_HEADER_SIZE = 4
MAX_CHUNK_DATA = TUNNEL_PAYLOAD_SIZE - CHUNK_HEADER_SIZE  # 124

# Custom TUNNEL payload_type so both ends (and nobody else) recognise our data.
# The 200..255 band of MAV_TUNNEL_PAYLOAD_TYPE is reserved for private use.
TUNNEL_PAYLOAD_TYPE = 200


@dataclass
class Waypoint:
    lat: float  # degrees
    lon: float  # degrees
    alt: float  # metres, relative

    def as_dict(self) -> dict:
        return {"lat": self.lat, "lon": self.lon, "alt": self.alt}


# ---- CRC16-CCITT (0x1021, init 0xFFFF) --------------------------------------
def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


# ---- blob (de)serialisation -------------------------------------------------
def pack_mission(waypoints: List[Waypoint]) -> bytes:
    """Serialise a waypoint list into a CRC-protected mission blob."""
    if len(waypoints) > 255:
        raise ValueError(f"too many waypoints: {len(waypoints)} (max 255)")
    body = struct.pack("<BB", MSG_MISSION, len(waypoints))
    for wp in waypoints:
        lat_e7 = int(round(wp.lat * 1e7))
        lon_e7 = int(round(wp.lon * 1e7))
        alt_m = max(0, min(65535, int(round(wp.alt))))
        body += struct.pack("<iiH", lat_e7, lon_e7, alt_m)
    crc = crc16_ccitt(body)
    return body + struct.pack("<H", crc)


def unpack_mission(blob: bytes) -> List[Waypoint]:
    """Parse and CRC-verify a mission blob. Raises ValueError on any problem."""
    if len(blob) < 4:
        raise ValueError("mission blob too short")
    body, crc_rx = blob[:-2], struct.unpack("<H", blob[-2:])[0]
    if crc16_ccitt(body) != crc_rx:
        raise ValueError("mission CRC mismatch")
    msg_type, count = struct.unpack("<BB", body[:2])
    if msg_type != MSG_MISSION:
        raise ValueError(f"unexpected message type 0x{msg_type:02X}")
    expected = 2 + count * 10
    if len(body) != expected:
        raise ValueError(f"length mismatch: got {len(body)}, expected {expected}")
    waypoints: List[Waypoint] = []
    off = 2
    for _ in range(count):
        lat_e7, lon_e7, alt_m = struct.unpack("<iiH", body[off:off + 10])
        waypoints.append(Waypoint(lat_e7 / 1e7, lon_e7 / 1e7, float(alt_m)))
        off += 10
    return waypoints


def mission_crc(blob: bytes) -> int:
    """The trailing CRC16 of a blob (used for ACK verification)."""
    return struct.unpack("<H", blob[-2:])[0]


# ---- chunking / reassembly --------------------------------------------------
def chunk_blob(blob: bytes) -> List[bytes]:
    """Split a blob into TUNNEL-sized chunks, each a 128-byte padded payload."""
    total = max(1, (len(blob) + MAX_CHUNK_DATA - 1) // MAX_CHUNK_DATA)
    chunks: List[bytes] = []
    for seq in range(total):
        data = blob[seq * MAX_CHUNK_DATA:(seq + 1) * MAX_CHUNK_DATA]
        header = struct.pack("<BBBB", MSG_MISSION, seq, total, len(data))
        payload = header + data
        payload += b"\x00" * (TUNNEL_PAYLOAD_SIZE - len(payload))  # pad to 128
        chunks.append(payload)
    return chunks


def make_ack(count: int, crc: int, ok: bool = True) -> bytes:
    """Build a one-chunk ACK/NACK payload (padded to 128 bytes)."""
    ftype = MSG_ACK if ok else MSG_NACK
    data = struct.pack("<BH", count & 0xFF, crc & 0xFFFF)
    payload = struct.pack("<BBBB", ftype, 0, 1, len(data)) + data
    return payload + b"\x00" * (TUNNEL_PAYLOAD_SIZE - len(payload))


def make_cmd(cmd_id: int, param: float = 0.0) -> bytes:
    """Build a one-chunk operator-command payload (padded to 128 bytes)."""
    data = struct.pack("<Bf", cmd_id & 0xFF, float(param))
    payload = struct.pack("<BBBB", MSG_CMD, 0, 1, len(data)) + data
    return payload + b"\x00" * (TUNNEL_PAYLOAD_SIZE - len(payload))


def parse_frame(payload: bytes) -> Optional[dict]:
    """Decode a single TUNNEL payload's chunk header. Returns None if invalid."""
    if len(payload) < CHUNK_HEADER_SIZE:
        return None
    ftype, seq, total, data_len = struct.unpack("<BBBB", payload[:CHUNK_HEADER_SIZE])
    if ftype not in (MSG_MISSION, MSG_ACK, MSG_NACK, MSG_STATUS, MSG_CMD):
        return None
    if data_len > MAX_CHUNK_DATA:
        return None
    data = payload[CHUNK_HEADER_SIZE:CHUNK_HEADER_SIZE + data_len]
    return {"type": ftype, "seq": seq, "total": total, "data": data}


class MissionReassembler:
    """Collects mission chunks until a full blob is received and verified."""

    def __init__(self) -> None:
        self._chunks: Dict[int, bytes] = {}
        self._total: Optional[int] = None

    def reset(self) -> None:
        self._chunks.clear()
        self._total = None

    def feed(self, payload: bytes) -> Optional[List[Waypoint]]:
        """Feed one TUNNEL payload. Returns waypoints once the blob completes."""
        frame = parse_frame(payload)
        if frame is None or frame["type"] != MSG_MISSION:
            return None
        # A fresh transfer (seq 0) resets any partial state.
        if frame["seq"] == 0 and self._total != frame["total"]:
            self.reset()
        self._total = frame["total"]
        self._chunks[frame["seq"]] = frame["data"]
        if len(self._chunks) != self._total:
            return None
        blob = b"".join(self._chunks[i] for i in range(self._total))
        waypoints = unpack_mission(blob)  # raises on CRC/format error
        self.reset()
        return waypoints
