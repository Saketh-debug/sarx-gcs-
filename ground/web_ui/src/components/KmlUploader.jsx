import { useRef, useState } from 'react'
import { parseKml } from '../utils/kml.js'

// Upload a Mission Planner KML; parses to waypoints and hands them up.
export default function KmlUploader({ onLoaded, cruiseAlt }) {
  const inputRef = useRef(null)
  const [error, setError] = useState('')

  const handleFile = async (file) => {
    setError('')
    try {
      const text = await file.text()
      let wps = parseKml(text)
      if (wps.length === 0) throw new Error('No coordinates found in KML')
      // apply cruise-altitude default where the KML had none
      wps = wps.map((w) => ({ ...w, alt: w.alt > 0 ? w.alt : cruiseAlt }))
      onLoaded(wps)
    } catch (e) {
      setError(e.message || 'Failed to parse KML')
    }
  }

  return (
    <div className="panel">
      <h3>1 · Load mission</h3>
      <button className="btn" onClick={() => inputRef.current?.click()}>
        Upload Mission Planner KML
      </button>
      <input
        ref={inputRef}
        type="file"
        accept=".kml,.xml,application/vnd.google-earth.kml+xml"
        style={{ display: 'none' }}
        onChange={(e) => { if (e.target.files[0]) handleFile(e.target.files[0]); e.target.value = '' }}
      />
      <p className="hint">…or click the map to drop waypoints manually.</p>
      {error && <p className="error">{error}</p>}
    </div>
  )
}
