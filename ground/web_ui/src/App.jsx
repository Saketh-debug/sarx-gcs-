import { useState, useCallback } from 'react'
import MapView from './components/MapView.jsx'
import KmlUploader from './components/KmlUploader.jsx'
import WaypointEditor from './components/WaypointEditor.jsx'
import MissionControls from './components/MissionControls.jsx'
import StatusPanel from './components/StatusPanel.jsx'
import { useRouterSocket } from './utils/ws.js'

export default function App() {
  const [waypoints, setWaypoints] = useState([])
  const [cruiseAlt, setCruiseAlt] = useState(15)
  const { connected, position, heartbeat, status, ack, send } = useRouterSocket()

  const addWaypoint = useCallback((lat, lon) => {
    setWaypoints((wps) => [...wps, { lat, lon, alt: cruiseAlt }])
  }, [cruiseAlt])

  const moveWaypoint = useCallback((i, lat, lon) => {
    setWaypoints((wps) => wps.map((w, k) => (k === i ? { ...w, lat, lon } : w)))
  }, [])

  const deleteWaypoint = useCallback((i) => {
    setWaypoints((wps) => wps.filter((_, k) => k !== i))
  }, [])

  return (
    <div className="app">
      <div className="map-wrap">
        <MapView
          waypoints={waypoints}
          position={position}
          onAddWaypoint={addWaypoint}
          onMoveWaypoint={moveWaypoint}
          onDeleteWaypoint={deleteWaypoint}
        />
      </div>
      <aside className="sidebar">
        <h1>Companion GCS</h1>
        <KmlUploader onLoaded={setWaypoints} cruiseAlt={cruiseAlt} />
        <WaypointEditor
          waypoints={waypoints}
          cruiseAlt={cruiseAlt}
          setCruiseAlt={setCruiseAlt}
          onChange={setWaypoints}
          onClear={() => setWaypoints([])}
        />
        <MissionControls
          waypoints={waypoints}
          connected={connected}
          send={send}
          ackOk={ack?.ok}
        />
        <StatusPanel connected={connected} heartbeat={heartbeat} status={status} ack={ack} />
      </aside>
    </div>
  )
}
