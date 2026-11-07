"""Status transmitter: push FSM/telemetry summaries back to the ground UI.

Encodes a small JSON object into a single STATUS TUNNEL frame. Kept short so it
always fits in one 124-byte chunk. Rate-limited by the caller.
"""

from __future__ import annotations

import json
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "common"))
import mission_codec as mc  # noqa: E402


def send_status(mav, state: str, wp_index: int, wp_total: int,
                note: str = "") -> None:
    obj = {"s": state, "i": wp_index, "n": wp_total}
    if note:
        obj["m"] = note[:40]
    data = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    if len(data) > mc.MAX_CHUNK_DATA:
        data = data[:mc.MAX_CHUNK_DATA]
    payload = struct.pack("<BBBB", mc.MSG_STATUS, 0, 1, len(data)) + data
    payload += b"\x00" * (mc.TUNNEL_PAYLOAD_SIZE - len(payload))
    mav.send_tunnel(payload)
