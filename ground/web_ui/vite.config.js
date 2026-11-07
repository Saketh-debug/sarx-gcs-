import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// Dev server on 5173. The UI talks to the ground router over ws://127.0.0.1:8080.
export default defineConfig({
  plugins: [react()],
  server: { port: 5173, host: '127.0.0.1' },
})
