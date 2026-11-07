import { useEffect, useRef, useState, useCallback } from 'react'

// Connects to the ground router's WebSocket, auto-reconnecting. Exposes the
// latest telemetry/status and a send() for mission + command messages.
export function useRouterSocket(url = 'ws://127.0.0.1:8080') {
  const [connected, setConnected] = useState(false)
  const [position, setPosition] = useState(null)   // { lat, lon, alt, hdg }
  const [heartbeat, setHeartbeat] = useState(null)  // { armed, mode }
  const [status, setStatus] = useState(null)        // { s, i, n, m }
  const [ack, setAck] = useState(null)              // { ok, count, crc }
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

  return { connected, position, heartbeat, status, ack, send }
}
