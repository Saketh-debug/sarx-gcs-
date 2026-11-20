import { useState, useCallback } from 'react'
import MapView from './components/MapView.jsx'
import KmlUploader from './components/KmlUploader.jsx'
import WaypointEditor from './components/WaypointEditor.jsx'
import MissionControls from './components/MissionControls.jsx'
import StatusPanel from './components/StatusPanel.jsx'
import DiagnosticsView from './components/DiagnosticsView.jsx'
import { useRouterSocket } from './utils/ws.js'

export default function App() {
  const [currentTab, setCurrentTab] = useState('mission') // 'mission' | 'diagnostics'
  const [waypoints, setWaypoints] = useState([])
  const [cruiseAlt, setCruiseAlt] = useState(15)
  const {
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
    sendPing,
  } = useRouterSocket()

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
    <div className="app-container">
      {/* Top Header Navigation Bar */}
      <header className="top-nav">
        <div className="brand-area">
          <h1>SARX COMPANION GCS</h1>
          <span className={`status-tag ${connected ? 'online' : 'offline'}`}>
            <span className="dot" style={{ width: 6, height: 6, borderRadius: '50%', background: connected ? '#4caf50' : '#d9534f' }}></span>
            {connected ? 'ROUTER CONNECTED' : 'ROUTER OFFLINE'}
          </span>
          {heartbeat && (
            <span className="status-tag online">
              {heartbeat.mode} {heartbeat.armed ? '· ARMED' : ''}
            </span>
          )}
        </div>
        <nav className="nav-tabs">
          <button
            className={`nav-tab ${currentTab === 'mission' ? 'active' : ''}`}
            onClick={() => setCurrentTab('mission')}
          >
            🗺️ Mission Operations
          </button>
          <button
            className={`nav-tab ${currentTab === 'diagnostics' ? 'active' : ''}`}
            onClick={() => setCurrentTab('diagnostics')}
          >
            🛠️ Hardware Diagnostics & Link Tests
          </button>
        </nav>
      </header>

      {/* Main Content Area */}
      {currentTab === 'mission' ? (
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
            <KmlUploader onLoaded={setWaypoints} cruiseAlt={cruiseAlt} />
            <WaypointEditor
              waypoints={waypoints}
              position={position}
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
      ) : (
        <DiagnosticsView
          connected={connected}
          heartbeat={heartbeat}
          status={status}
          ack={ack}
          gps={gps}
          sysStatus={sysStatus}
          attitude={attitude}
          radio={radio}
          linkStats={linkStats}
          pong={pong}
          send={send}
          sendPing={sendPing}
        />
      )}
    </div>
  )
}
