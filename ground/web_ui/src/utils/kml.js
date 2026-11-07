// Parse a Mission Planner KML/KMZ-exported waypoint plan into an ordered
// waypoint list. KML stores coordinates as "lon,lat,alt" (note the order).

// Returns [{ lat, lon, alt }] in document order. Altitude 0/missing is kept as
// 0 so the caller can apply the cruise-altitude default.
export function parseKml(text) {
  const doc = new DOMParser().parseFromString(text, 'application/xml')
  if (doc.querySelector('parsererror')) {
    throw new Error('Not a valid KML/XML file')
  }
  const waypoints = []
  // Every <coordinates> node (Point, LineString, Polygon ring) contributes
  // its coordinate tuples, in order.
  for (const node of doc.getElementsByTagName('coordinates')) {
    const raw = (node.textContent || '').trim()
    if (!raw) continue
    for (const tuple of raw.split(/\s+/)) {
      const parts = tuple.split(',')
      if (parts.length < 2) continue
      const lon = parseFloat(parts[0])
      const lat = parseFloat(parts[1])
      const alt = parts.length >= 3 ? parseFloat(parts[2]) : 0
      if (Number.isFinite(lat) && Number.isFinite(lon)) {
        waypoints.push({ lat, lon, alt: Number.isFinite(alt) ? alt : 0 })
      }
    }
  }
  return dedupeConsecutive(waypoints)
}

// MP sometimes repeats the first/last point to close a path; drop immediate dups.
function dedupeConsecutive(wps) {
  const out = []
  for (const wp of wps) {
    const prev = out[out.length - 1]
    if (prev && Math.abs(prev.lat - wp.lat) < 1e-9 && Math.abs(prev.lon - wp.lon) < 1e-9) {
      continue
    }
    out.push(wp)
  }
  return out
}

// Haversine total path length in metres (for the preview readout).
export function pathLengthMeters(wps) {
  const R = 6371000
  let total = 0
  for (let i = 1; i < wps.length; i++) {
    const a = wps[i - 1], b = wps[i]
    const dLat = (b.lat - a.lat) * Math.PI / 180
    const dLon = (b.lon - a.lon) * Math.PI / 180
    const la1 = a.lat * Math.PI / 180, la2 = b.lat * Math.PI / 180
    const h = Math.sin(dLat / 2) ** 2 + Math.cos(la1) * Math.cos(la2) * Math.sin(dLon / 2) ** 2
    total += 2 * R * Math.asin(Math.min(1, Math.sqrt(h)))
  }
  return total
}
