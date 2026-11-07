#!/usr/bin/env bash
# Launch ArduPilot SITL with the MAVLink outputs this project expects.
#
#   14550 -> Mission Planner (connect MP directly to SITL in simulation)
#   14551 -> ground_router.py  (acts as the "ESP-NOW link")
#   14553 -> autonomy main.py  (the companion, in place of mavlink-router)
#
# MAVProxy (started by sim_vehicle.py) acts as the hub and relays MAVLink
# between these links, so a TUNNEL injected by the router reaches the autonomy
# and the ACK/STATUS comes back.
#
# Prereq: ArduPilot source + sim_vehicle.py on PATH (see ardupilot.org SITL setup).
#
# Then, in three more terminals:
#   1) python ground/router/ground_router.py --link udpin:127.0.0.1:14551
#   2) cd ground/web_ui && npm run dev          # http://localhost:5173
#   3) python onboard_pi/autonomy/main.py --conn udpin:127.0.0.1:14553
#
# Connect Mission Planner (optional) to UDP 127.0.0.1:14550.

set -e
VEHICLE=${VEHICLE:-ArduCopter}
FRAME=${FRAME:-quad}
LOCATION=${LOCATION:-}   # e.g. LOCATION=CMAC ./sitl_runner.sh

ARGS="-v $VEHICLE -f $FRAME --console --map \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14553"

if [ -n "$LOCATION" ]; then
  ARGS="$ARGS -L $LOCATION"
fi

echo "sim_vehicle.py $ARGS"
exec sim_vehicle.py $ARGS
