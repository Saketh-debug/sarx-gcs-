import { useState } from 'react'

// Transmit the mission, then an explicit Start gate (never auto-arms). Abort
// stops autonomy (vehicle hovers in GUIDED / pilot can take over any time).
export default function MissionControls({ waypoints, connected, send, ackOk }) {
  const [sent, setSent] = useState(false)
  const canTransmit = connected && waypoints.length >= 1

  const transmit = () => {
    if (send({ kind: 'mission', waypoints })) setSent(true)
  }
  const start = () => send({ kind: 'command', cmd: 'start' })
  const abort = () => send({ kind: 'command', cmd: 'abort' })

  return (
    <div className="panel">
      <h3>3 · Dispatch</h3>
      <button className="btn primary" disabled={!canTransmit} onClick={transmit}>
        Transmit mission
      </button>
      <div className="btn-row">
        <button className="btn go" disabled={!sent || !ackOk} onClick={start}>Start</button>
        <button className="btn stop" disabled={!connected} onClick={abort}>Abort</button>
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
