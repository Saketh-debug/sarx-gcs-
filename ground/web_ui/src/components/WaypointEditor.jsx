import { pathLengthMeters } from '../utils/kml.js'

// Editable waypoint list: per-wp altitude, reorder, delete, set-all altitude.
export default function WaypointEditor({ waypoints, position, cruiseAlt, setCruiseAlt, onChange, onClear }) {
  const setAlt = (i, alt) => {
    const next = waypoints.slice()
    next[i] = { ...next[i], alt: Number(alt) }
    onChange(next)
  }
  const move = (i, dir) => {
    const j = i + dir
    if (j < 0 || j >= waypoints.length) return
    const next = waypoints.slice()
    ;[next[i], next[j]] = [next[j], next[i]]
    onChange(next)
  }
  const del = (i) => onChange(waypoints.filter((_, k) => k !== i))
  const applyAllAlt = () => onChange(waypoints.map((w) => ({ ...w, alt: cruiseAlt })))

  const generateNearDrone = () => {
    if (!position || !position.lat || !position.lon) return
    const dLat = 40 / 111320
    const dLon = 40 / (111320 * Math.cos((position.lat * Math.PI) / 180))
    const lat = position.lat
    const lon = position.lon
    const box = [
      { lat: Number((lat + dLat).toFixed(7)), lon: Number(lon.toFixed(7)), alt: cruiseAlt },
      { lat: Number((lat + dLat).toFixed(7)), lon: Number((lon + dLon).toFixed(7)), alt: cruiseAlt },
      { lat: Number(lat.toFixed(7)), lon: Number((lon + dLon).toFixed(7)), alt: cruiseAlt },
      { lat: Number(lat.toFixed(7)), lon: Number(lon.toFixed(7)), alt: cruiseAlt },
    ]
    onChange(box)
  }

  const dist = pathLengthMeters(waypoints)

  return (
    <div className="panel">
      <h3>2 · Waypoints ({waypoints.length})</h3>
      <div className="alt-row">
        <label>Cruise alt (m)
          <input type="number" min="1" max="120" value={cruiseAlt}
            onChange={(e) => setCruiseAlt(Number(e.target.value))} />
        </label>
        <button className="btn small" onClick={applyAllAlt} disabled={!waypoints.length}>
          Apply to all
        </button>
      </div>
      <div style={{ marginTop: '8px', display: 'flex', gap: '6px' }}>
        <button className="btn small primary" onClick={generateNearDrone} disabled={!position}>
          📍 Pattern at Drone
        </button>
        {waypoints.length > 0 && (
          <button className="btn small ghost" onClick={onClear}>Clear</button>
        )}
      </div>
      <p className="hint">Path length ≈ {dist.toFixed(0)} m · right-click a pin to delete</p>

      <div className="wp-list">
        {waypoints.map((w, i) => (
          <div className="wp-row" key={i}>
            <span className={'wp-num' + (i === waypoints.length - 1 ? ' land' : '')}>
              {i === waypoints.length - 1 ? 'LAND' : i + 1}
            </span>
            <span className="wp-coord">{w.lat.toFixed(6)}, {w.lon.toFixed(6)}</span>
            <input className="wp-alt" type="number" value={Math.round(w.alt)}
              onChange={(e) => setAlt(i, e.target.value)} />
            <button className="icon" title="up" onClick={() => move(i, -1)}>▲</button>
            <button className="icon" title="down" onClick={() => move(i, 1)}>▼</button>
            <button className="icon del" title="delete" onClick={() => del(i)}>✕</button>
          </div>
        ))}
      </div>
      {waypoints.length > 0 && (
        <button className="btn small ghost" onClick={onClear}>Clear all</button>
      )}
    </div>
  )
}
