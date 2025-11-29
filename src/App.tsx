import { useEffect, useRef, useState } from 'react'
import { SceneController, type PlannerStats } from './logic/SceneController'
import './App.css'

function App() {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const controllerRef = useRef<SceneController | null>(null)
  const [stats, setStats] = useState<PlannerStats | null>(null)
  const [scenario, setScenario] = useState<'easy' | 'medium' | 'hard'>('medium')
  const [debugMode, setDebugMode] = useState(false)

  useEffect(() => {
    if (!canvasRef.current) return
    const controller = new SceneController(canvasRef.current)
    controllerRef.current = controller
    controller.onStatsUpdate = setStats

    const onResize = () =>
      controller.resize(window.innerWidth, window.innerHeight)
    window.addEventListener('resize', onResize)
    return () => window.removeEventListener('resize', onResize)
  }, [])

  // Handlers
  const handleScenarioChange = (type: 'easy' | 'medium' | 'hard') => {
    setScenario(type)
    controllerRef.current?.setScenario(type)
  }

  const toggleDebug = () => {
    const newVal = !debugMode
    setDebugMode(newVal)
    controllerRef.current?.toggleCollisionDebug(newVal)
  }

  return (
    <div className="app-container">
      <canvas ref={canvasRef} className="webgl-canvas" />

      <div className="ui-overlay">
        {/* Left Panel: The Essay */}
        <div className="essay-panel">
          <h1>Robotic Path Planning</h1>
          <p className="subtitle">From Inverse Kinematics to RRT-Connect</p>

          <div className="explanation">
            <h3>The Challenge</h3>
            <p>
              In a perfect world, we just move the hand to the target (Inverse
              Kinematics). But in the real world, obstacles create "shadows" in
              the robot's configuration space.
            </p>
            <h3>The Solution</h3>
            <p>
              This demo uses <strong>Bi-Directional RRT (RRT-Connect)</strong>.
              Instead of blindly searching, we grow two search trees: one from
              the robot and one from the target. They aggressively try to meet
              in the middle.
            </p>
          </div>

          <div className="controls-section">
            <h3>Robot Config</h3>
            <div className="button-group">
              <button onClick={() => controllerRef.current?.addJoint()}>
                + Add Joint
              </button>
              <button onClick={() => controllerRef.current?.resetJoints()}>
                Reset
              </button>
            </div>

            <h3>Select Scenario</h3>
            <div className="button-group">
              <button
                className={scenario === 'easy' ? 'active' : ''}
                onClick={() => handleScenarioChange('easy')}
              >
                1. Free Space
              </button>
              <button
                className={scenario === 'medium' ? 'active' : ''}
                onClick={() => handleScenarioChange('medium')}
              >
                2. The Wall
              </button>
              <button
                className={scenario === 'hard' ? 'active' : ''}
                onClick={() => handleScenarioChange('hard')}
              >
                3. Deep Reach
              </button>
            </div>
            <p className="hint">
              {scenario === 'easy' && 'Pure IK works here. No planning needed.'}
              {scenario === 'medium' &&
                'IK fails (hits wall). RRT finds the path over.'}
              {scenario === 'hard' &&
                'Complex geometry. RRT explores thousands of nodes.'}
            </p>
          </div>
        </div>

        {/* Right Panel: The Engineer's Dashboard */}
        <div className="dashboard-panel">
          <h3>Real-Time Telemetry</h3>

          <div className="stat-card">
            <div className="stat-row">
              <span className="label">Algorithm</span>
              <span className="value">RRT-Connect</span>
            </div>
            <div className="stat-row">
              <span className="label">Status</span>
              <span
                className={`value ${stats?.success ? 'success' : 'pending'}`}
              >
                {stats ? (stats.success ? 'CONVERGED' : 'PLANNING') : 'IDLE'}
              </span>
            </div>
            <div className="stat-row">
              <span className="label">Compute Time</span>
              <span className="value">{stats?.time || 0} ms</span>
            </div>
            <div className="stat-row">
              <span className="label">Nodes Visited</span>
              <span className="value">{stats?.nodes || 0}</span>
            </div>
          </div>

          <div className="debug-section">
            <label className="toggle-row">
              <input
                type="checkbox"
                checked={debugMode}
                onChange={toggleDebug}
              />
              <span>Show Collision Hitboxes (Engineer View)</span>
            </label>
            <p className="tiny-text">
              Visualizes the raw math capsules used for high-frequency collision
              checking (10k checks/sec).
            </p>
          </div>
        </div>
      </div>
    </div>
  )
}

export default App
