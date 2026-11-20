import { useEffect, useRef, useState, useCallback } from 'react'

// Connects to the ground router's WebSocket, auto-reconnecting. Exposes the
// latest telemetry/status and a send() for mission + command messages.
export function useRouterSocket(url = 'ws://127.0.0.1:8080') {
  const [connected, setConnected] = useState(false)
  const [position, setPosition] = useState(null)   // { lat, lon, alt, hdg }
  const [heartbeat, setHeartbeat] = useState(null)  // { armed, mode, sys_id, comp_id }
  const [status, setStatus] = useState(null)        // { s, i, n, m }
  const [ack, setAck] = useState(null)              // { ok, count, crc }
  const [gps, setGps] = useState(null)              // { fix_type, satellites, hdop }
  const [sysStatus, setSysStatus] = useState(null)  // { voltage, current, battery_remaining }
  const [attitude, setAttitude] = useState(null)    // { roll, pitch, yaw }
  const [radio, setRadio] = useState(null)          // { rssi, remrssi, rxerrors, fixed, noise }
  const [linkStats, setLinkStats] = useState(null)  // { rate_hz, total_packets, hb_age }
  const [pong, setPong] = useState(null)            // { rtt_ms, t }
  const wsRef = useRef(null)

  useEffect(() => {
    let closed = false
    let retry

    const connect = () => {
      const ws = new WebSocket(url)
      wsRef.current = ws
      ws.onopen = () => setConnected(true)
      ws.onclose = () => {
        setConnected(false)
        if (!closed) retry = setTimeout(connect, 1500)
      }
      ws.onerror = () => ws.close()
      ws.onmessage = (ev) => {
        let msg
        try { msg = JSON.parse(ev.data) } catch { return }
        switch (msg.kind) {
          case 'position': setPosition(msg); break
          case 'heartbeat': setHeartbeat(msg); break
          case 'status': setStatus(msg); break
          case 'ack': setAck(msg); break
          case 'gps': setGps(msg); break
          case 'sys_status': setSysStatus(msg); break
          case 'attitude': setAttitude(msg); break
          case 'radio': setRadio(msg); break
          case 'link_stats': setLinkStats(msg); break
          case 'pong': setPong(msg); break
          default: break
        }
      }
    }
    connect()
    return () => { closed = true; clearTimeout(retry); wsRef.current?.close() }
  }, [url])

  const send = useCallback((obj) => {
    const ws = wsRef.current
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(obj))
      return true
    }
    return false
  }, [])

  const sendPing = useCallback(() => {
    return send({ kind: 'ping' })
  }, [send])

  return {
    connected,
    position,
    heartbeat,
    status,
    ack,
    gps,
    sysStatus,
    attitude,
    radio,
    linkStats,
    pong,
    send,
    sendPing
  }
}
