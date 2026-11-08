// Live link + FSM status. Mission Planner remains the primary GCS; this is a
// compact companion-status readout.
const FSM_LABEL = {
  BOOT: 'Booting', IDLE: 'Idle', PREARM: 'Pre-arm checks', TAKEOFF: 'Taking off',
  HOVER: 'Hovering (Guided)', EXPLORE: 'Flying waypoints', NAV_LAND: 'To landing point', LAND: 'Landing',
}

export default function StatusPanel({ connected, heartbeat, status, ack }) {
  return (
    <div className="panel">
      <h3>Status</h3>
      <div className="stat">
        <span>Router link</span>
        <b className={connected ? 'ok' : 'bad'}>{connected ? 'connected' : 'offline'}</b>
      </div>
      <div className="stat">
        <span>Vehicle</span>
        <b>{heartbeat ? `${heartbeat.mode}${heartbeat.armed ? ' · ARMED' : ''}` : '—'}</b>
      </div>
      <div className="stat">
        <span>Autonomy</span>
        <b>{status ? (FSM_LABEL[status.s] || status.s) : '—'}</b>
      </div>
      {status && status.n > 0 && (
        <div className="stat">
          <span>Progress</span>
          <b>{status.s === 'NAV_LAND' ? `Landing (${status.n}/${status.n})` : `wp ${Math.min(status.i + 1, status.n)} / ${status.n}`}</b>
        </div>
      )}
      {status && status.m && <p className="hint">{status.m}</p>}
      {ack && (
        <p className={ack.ok ? 'ok' : 'error'}>
          {ack.ok ? `Mission loaded ✓ (${ack.count} wp)` : 'Mission rejected ✗'}
        </p>
      )}
    </div>
  )
}
