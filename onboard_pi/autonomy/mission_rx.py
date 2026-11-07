"""Mission receiver: reassembles TUNNEL chunks into a waypoint list and ACKs.

The MavInterface hands us each TUNNEL payload addressed to our custom
payload_type. We feed them to the shared MissionReassembler; when a full,
CRC-valid mission arrives we store it and transmit an ACK back over the link.
Bad CRC / malformed transfers trigger a NACK.
"""

from __future__ import annotations

import os
import sys
import threading
from typing import List, Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "common"))
import mission_codec as mc  # noqa: E402


class MissionReceiver:
    def __init__(self, mav) -> None:
        self._mav = mav
        self._asm = mc.MissionReassembler()
        self._lock = threading.Lock()
        self._pending: Optional[List[mc.Waypoint]] = None
        self._new_event = threading.Event()

    def on_tunnel(self, payload: bytes) -> None:
        """Called from the MAVLink pump thread for each of our TUNNEL frames."""
        frame = mc.parse_frame(payload)
        if frame is None or frame["type"] != mc.MSG_MISSION:
            return
        try:
            waypoints = self._asm.feed(payload)
        except ValueError as e:  # CRC / format failure on a completed transfer
            print(f"[mission_rx] rejected mission: {e}")
            self._mav.send_tunnel(mc.make_ack(0, 0, ok=False))
            self._asm.reset()
            return
        if waypoints is None:
            return  # still collecting chunks
        blob = mc.pack_mission(waypoints)
        print(f"[mission_rx] accepted mission: {len(waypoints)} waypoints")
        self._mav.send_tunnel(mc.make_ack(len(waypoints), mc.mission_crc(blob), ok=True))
        with self._lock:
            self._pending = waypoints
            self._new_event.set()

    def take_new_mission(self) -> Optional[List[mc.Waypoint]]:
        """Non-blocking: return a freshly received mission once, else None."""
        with self._lock:
            if self._new_event.is_set():
                self._new_event.clear()
                return self._pending
        return None

    def has_mission(self) -> bool:
        with self._lock:
            return self._pending is not None
