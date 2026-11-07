import { useEffect } from 'react'
import { MapContainer, TileLayer, Marker, Polyline, useMap, useMapEvents } from 'react-leaflet'
import L from 'leaflet'

// Numbered waypoint pin (last one badged as LAND).
function wpIcon(index, isLast) {
  const label = isLast ? 'L' : String(index + 1)
  const bg = isLast ? '#d9534f' : '#2d6cdf'
  return L.divIcon({
    className: 'wp-icon',
    html: `<div class="wp-pin" style="background:${bg}">${label}</div>`,
    iconSize: [26, 26],
    iconAnchor: [13, 13],
  })
}

function droneIcon(hdg) {
  const rot = hdg == null ? 0 : hdg
  return L.divIcon({
    className: 'drone-icon',
    html: `<div class="drone-arrow" style="transform:rotate(${rot}deg)">▲</div>`,
    iconSize: [28, 28],
    iconAnchor: [14, 14],
  })
}

function ClickToAdd({ onAdd }) {
  useMapEvents({ click(e) { onAdd(e.latlng.lat, e.latlng.lng) } })
  return null
}

// Pan to the drone the first time we get a fix.
function FollowDrone({ position }) {
  const map = useMap()
  useEffect(() => {
    if (position && !map.__panned) {
      map.setView([position.lat, position.lon], 18)
      map.__panned = true
    }
  }, [position, map])
  return null
}

export default function MapView({ waypoints, position, onAddWaypoint, onMoveWaypoint, onDeleteWaypoint }) {
  const center = position ? [position.lat, position.lon] : [17.4123, 78.1234]
  const line = waypoints.map((w) => [w.lat, w.lon])

  return (
    <MapContainer center={center} zoom={16} className="map">
      <TileLayer
        attribution='&copy; OpenStreetMap contributors'
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />
      <ClickToAdd onAdd={onAddWaypoint} />
      <FollowDrone position={position} />

      {line.length > 1 && <Polyline positions={line} color="#2d6cdf" weight={3} opacity={0.8} />}

      {waypoints.map((w, i) => (
        <Marker
          key={i}
          position={[w.lat, w.lon]}
          draggable
          icon={wpIcon(i, i === waypoints.length - 1)}
          eventHandlers={{
            dragend: (e) => { const p = e.target.getLatLng(); onMoveWaypoint(i, p.lat, p.lng) },
            contextmenu: () => onDeleteWaypoint(i),
          }}
        />
      ))}

      {position && <Marker position={[position.lat, position.lon]} icon={droneIcon(position.hdg)} />}
    </MapContainer>
  )
}
