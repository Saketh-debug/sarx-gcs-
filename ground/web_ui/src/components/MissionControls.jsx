import { useState } from 'react'

export default function MissionControls({ waypoints, connected, send, ackOk }) {
  const [sent, setSent] = useState(false)
  const [takeoffAlt, setTakeoffAlt] = useState(15)
  const canTransmit = connected && waypoints.length >= 1

  const transmit = () => {
    if (send({ kind: 'mission', waypoints })) setSent(true)
  }
  const start = () => send({ kind: 'command', cmd: 'start' })
  const abort = () => send({ kind: 'command', cmd: 'abort' })
  const takeoff = () => send({ kind: 'command', cmd: 'takeoff', alt: Number(takeoffAlt) })
  const land = () => send({ kind: 'command', cmd: 'land' })

  return (
    <div className="panel">
      <h3>3 · Direct Controls</h3>
      <div className="alt-row" style={{ marginBottom: '8px' }}>
        <label>Takeoff Alt (m)
          <input type="number" min="1" max="120" value={takeoffAlt}
            onChange={(e) => setTakeoffAlt(Number(e.target.value))} />
        </label>
        <button className="btn primary small" disabled={!connected} onClick={takeoff}>
          🛫 Takeoff
        </button>
        <button className="btn stop small" disabled={!connected} onClick={land}>
          🛬 Land
        </button>
      </div>

      <h3 style={{ marginTop: '16px' }}>4 · Mission Dispatch</h3>
      <button className="btn primary" disabled={!canTransmit} onClick={transmit}>
        Transmit mission
      </button>
      <div className="btn-row">
        <button className="btn go" disabled={!sent || !ackOk} onClick={start}>
          Start Mission
        </button>
        <button className="btn stop" disabled={!connected} onClick={abort}>
          Abort / Hold
        </button>
      </div>
      {!connected && <p className="hint">Waiting for router link…</p>}
      {sent && !ackOk && <p className="hint">Mission sent — waiting for Pi ACK…</p>}
      <p className="hint small">
        Keep a hand on the RC flight-mode switch. Moving off GUIDED revokes
        autonomy instantly.
      </p>
    </div>
  )
}
