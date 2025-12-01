import { useEffect, useRef, useState } from 'react'
import { SceneController, type PlannerStats } from './logic/SceneController'
import './App.css'

function App() {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const controllerRef = useRef<SceneController | null>(null)
  const [stats, setStats] = useState<PlannerStats | null>(null)
  // Track Standard RRT stats specifically for comparison
  const [standardStats, setStandardStats] = useState<PlannerStats | null>(null)
  const [targetPos, setTargetPos] = useState<{
    x: number
    y: number
    z: number
  } | null>(null)
  const [scenario, setScenario] = useState<'easy' | 'medium' | 'hard'>('medium')
  const [debugMode, setDebugMode] = useState(false)

  // Obstacle Height (Default 2.0)
  const [obstacleHeight, setObstacleHeight] = useState(2.0)

  // 0: Intro, 1: Greedy IK, 2: Standard RRT, 3: RRT-Connect
  const [step, setStep] = useState<0 | 1 | 2 | 3>(0)

  // RRT Parameters
  const [rrtParams, setRrtParams] = useState({
    stepSize: 0.2,
    maxIter: 5000,
    goalBias: 0.1,
  })

  // 1. Initialize Scene
  useEffect(() => {
    if (!canvasRef.current) return
    const controller = new SceneController(canvasRef.current)
    controllerRef.current = controller
    controller.onStatsUpdate = (newStats) => {
      setStats(newStats)
      // Capture Standard RRT stats for comparison later (only if we are currently in Step 2)
      // Note: We need to check the current step Ref or just trust the logic.
      // Since this closure captures the initial state, we can't read 'step' directly here correctly if we don't add it to deps.
      // BUT: onStatsUpdate is assigned once.
      // Better approach: Pass the step to the stats object or update logic.
      // ACTUALLY: Let's update the callback in useEffect whenever step changes.
    }
    controller.onTargetMove = setTargetPos

    // Initial Scenario
    controller.setScenario(scenario)
    controller.setAlgorithm('rrt') // Default background mode
    controller.setInteraction(false) // Disable interaction for Intro

    const onResize = () =>
      controller.resize(window.innerWidth, window.innerHeight)
    window.addEventListener('resize', onResize)

    return () => window.removeEventListener('resize', onResize)
  }, []) // One-time init

  // Update the stats callback whenever step changes to capture context
  useEffect(() => {
    if (controllerRef.current) {
      controllerRef.current.onStatsUpdate = (newStats) => {
        setStats(newStats)
      }
    }
  }, [step])

  // 2. React to Step Changes
  const updateAlgorithmForStep = (newStep: number) => {
    if (!controllerRef.current) return

    // Reset stats on step change
    setStats(null)

    // Clear Standard Stats if going BACK before step 2
    if (newStep < 2) {
      setStandardStats(null)
    }

    if (newStep === 1) {
      // Switching to Greedy
      controllerRef.current.setAlgorithm('greedy', true)
      controllerRef.current.setInteraction(true)

      // Trigger Camera Intro Animation on Entry
      controllerRef.current.animateCameraIntro()

      // Reset Debug Mode
      setDebugMode(false)
      controllerRef.current.toggleCollisionDebug(false)

      // HARDCODED FAIL CASE:
      // Wall is at z=1. Robot at z=0.
      // Target at (0.5, 1.5, 2.0) -> Behind wall.
      // Robot tries to go straight -> Collision.
      controllerRef.current.setTargetPosition(0.39, 1.65, 2.0)
    } else if (newStep === 2) {
      // Switching to Standard RRT (The "Bushy" Tree)
      controllerRef.current.setAlgorithm('rrt-standard', true)
      controllerRef.current.setInteraction(true)

      // Apply WEAK defaults for "Failure" demo
      const weakParams = { stepSize: 0.05, maxIter: 5000, goalBias: 0.05 }
      setRrtParams(weakParams)
      controllerRef.current.updateRRTParams(weakParams)
    } else if (newStep === 3) {
      // Switching to RRT-Connect (The "Fast" Solution)
      controllerRef.current.setAlgorithm('rrt', true)
      controllerRef.current.setInteraction(true)

      // Apply WEAK defaults again to show improvement
      const weakParams = { stepSize: 0.05, maxIter: 5000, goalBias: 0.05 }
      setRrtParams(weakParams)
      controllerRef.current.updateRRTParams(weakParams)
    } else {
      // Intro
      controllerRef.current.setAlgorithm('rrt')
      controllerRef.current.setInteraction(false)
    }
  }

  // 3. React to Param Changes
  useEffect(() => {
    if (controllerRef.current && (step === 2 || step === 3)) {
      controllerRef.current.updateRRTParams(rrtParams)
    }
  }, [rrtParams, step])

  // Handlers
  const handleStepChange = (newStep: 0 | 1 | 2 | 3) => {
    // Capture Standard RRT stats when leaving Step 2 (Standard RRT)
    // This ensures we compare against the EXACT run the user just saw.
    if (step === 2 && stats) {
      setStandardStats(stats)
    }

    setStep(newStep)
    updateAlgorithmForStep(newStep)
  }

  const handleScenarioChange = (type: 'easy' | 'medium' | 'hard') => {
    setScenario(type)
    // Reset obstacle height for new scenario? Or keep it?
    // Let's reset to default 2.0 for consistency
    setObstacleHeight(2.0)
    controllerRef.current?.setScenario(type)
  }

  const handleObstacleHeightChange = (newHeight: number) => {
    setObstacleHeight(newHeight)
    controllerRef.current?.setObstacleHeight(newHeight)
  }

  const toggleDebug = () => {
    const newVal = !debugMode
    setDebugMode(newVal)
    controllerRef.current?.toggleCollisionDebug(newVal)
  }

  const handleParamChange = (key: keyof typeof rrtParams, value: number) => {
    setRrtParams((prev) => ({ ...prev, [key]: value }))
  }

  return (
    <div className="app-container">
      <canvas ref={canvasRef} className="webgl-canvas" />

      <div className="ui-overlay">
        {/* Left Panel: The Guide */}
        <div className="essay-panel">
          <div className="stepper-nav">
            <button
              className={step === 0 ? 'active' : ''}
              onClick={() => handleStepChange(0)}
            >
              Intro
            </button>
            <button
              className={step === 1 ? 'active' : ''}
              onClick={() => handleStepChange(1)}
            >
              1. Greedy
            </button>
            <button
              className={step === 2 ? 'active' : ''}
              onClick={() => handleStepChange(2)}
            >
              2. Std RRT
            </button>
            <button
              className={step === 3 ? 'active' : ''}
              onClick={() => handleStepChange(3)}
            >
              3. Connect
            </button>
          </div>

          {step === 0 && (
            <div className="step-content fade-in">
              <h1>Robotic Motion Planning</h1>
              <p className="subtitle">An Interactive Introduction</p>

              <div className="explanation">
                <h3>Welcome</h3>
                <p className="placeholder-text">
                  [PLACEHOLDER: INTRO TEXT]
                  <br />
                  Robotics is about movement. But how do we get from A to B
                  without hitting C? In this interactive tutorial, we will
                  explore the fundamental challenges of high-dimensional path
                  planning.
                  <br />
                  <br />
                  We start with the intuitive approach: just moving towards the
                  goal. Then we discover why that fails in complex environments.
                  Finally, we build up to RRT-Connect, a probabilistic algorithm
                  used in autonomous driving and industrial robotics.
                </p>
              </div>

              <button
                className="primary-btn"
                onClick={() => handleStepChange(1)}
              >
                Start Learning &rarr;
              </button>
            </div>
          )}

          {step === 1 && (
            <div className="step-content fade-in">
              <h1>1. The Greedy Approach</h1>
              <p className="subtitle">Inverse Kinematics (IK)</p>

              <div className="explanation">
                <h3>Intuition: Greedy Search</h3>
                <p className="placeholder-text">
                  [PLACEHOLDER: GREEDY INTUITION]
                  <br />
                  The simplest idea is "Greedy Descent". At every step, we try
                  to minimize the distance to the target. This is what Inverse
                  Kinematics (IK) does. It solves the mathematical equations to
                  put the hand exactly where you want it.
                  <br />
                  <br />
                  Try dragging the red target ball around. Notice how the robot
                  follows perfectly... until it doesn't.
                </p>

                <h3>Why it Fails</h3>
                <p className="placeholder-text">
                  [PLACEHOLDER: FAILURE CASES]
                  <br />
                  Greedy algorithms get stuck in "Local Minima". If there is a
                  wall between the robot and the goal, the greedy approach tries
                  to go *through* the wall because that is the shortest
                  mathematical path. It has no concept of "backing up" to go
                  around.
                </p>
              </div>

              <div className="controls-section">
                <p className="hint">
                  Drag the red ball to see the Greedy algorithm fail against the
                  wall.
                </p>
                <button
                  className="secondary-btn"
                  onClick={() => {
                    controllerRef.current?.setTargetPosition(2.24, 2.59, 2.0)
                  }}
                  style={{ marginTop: '10px' }}
                >
                  Move Target Behind Wall
                </button>
              </div>

              <div className="nav-buttons">
                <button onClick={() => handleStepChange(0)}>Back</button>
                <button
                  className="primary-btn"
                  onClick={() => handleStepChange(2)}
                >
                  Next: Exploration &rarr;
                </button>
              </div>
            </div>
          )}

          {step === 2 && (
            <div className="step-content fade-in">
              <h1>2. Standard RRT</h1>
              <p className="subtitle">Single Tree Exploration</p>

              <div className="explanation">
                <h3>Exploring the Unknown</h3>
                <p>
                  Standard RRT grows a single tree from the start. It explores
                  blindly.
                  <br />
                  <br />
                  <b>Try it:</b> Click "Run Planner".
                  <br />
                  Notice how "bushy" the tree is? It explores everywhere! But
                  for this "Behind the Wall" target, it will likely{' '}
                  <b>FAIL (botos will not snap to the target position)</b> or
                  time out. This is because it doesn't know where to go.
                  <br />
                  <br />
                  <b>Parameter Guide:</b>
                  <br />- <b>Step Size:</b> How far the robot reaches in each
                  step. Larger steps jump gaps but might hit walls.
                  <br />- <b>Iterations:</b> How many attempts to make. More
                  attempts = higher chance of success but slower.
                  <br />- <b>Goal Bias:</b> How often it tries to go straight to
                  the goal. Higher bias = faster but might get stuck.
                  <br />
                  <br />
                  <b>Still Failing?</b> If after setting all parameters to the
                  most favorable positions, it still doesn't find the target,
                  this means the configuration is very difficult given the angle
                  constraints.
                  <br />
                  Try adding a <b>Joint</b> (on the right hand side).
                </p>
              </div>

              <div className="controls-section">
                <p className="hint">
                  Parameters are set to "Weak" defaults. Tune them!
                </p>
                <div className="parameter-control">
                  <label>Step Size (Growth Rate): {rrtParams.stepSize}</label>
                  <input
                    type="range"
                    min="0.01"
                    max="0.5"
                    step="0.01"
                    value={rrtParams.stepSize}
                    onChange={(e) =>
                      handleParamChange('stepSize', parseFloat(e.target.value))
                    }
                  />
                </div>
                <div className="parameter-control">
                  <label>Max Iterations (Effort): {rrtParams.maxIter}</label>
                  <input
                    type="range"
                    min="1000"
                    max="10000"
                    step="1000"
                    value={rrtParams.maxIter}
                    onChange={(e) =>
                      handleParamChange('maxIter', parseInt(e.target.value))
                    }
                  />
                </div>
                <div className="parameter-control">
                  <label>Goal Bias: {rrtParams.goalBias}</label>
                  <input
                    type="range"
                    min="0"
                    max="1"
                    step="0.05"
                    value={rrtParams.goalBias}
                    onChange={(e) =>
                      handleParamChange('goalBias', parseFloat(e.target.value))
                    }
                  />
                </div>
              </div>

              <div className="nav-buttons">
                <button onClick={() => handleStepChange(1)}>Back</button>
                <button
                  className="run-btn"
                  onClick={() => controllerRef.current?.runPlanner()}
                  style={{
                    backgroundColor: '#2ecc71',
                    color: 'white',
                    fontWeight: 'bold',
                  }}
                >
                  ▶ Run Planner
                </button>
                <button
                  className="primary-btn"
                  onClick={() => handleStepChange(3)}
                >
                  Next: Optimization &rarr;
                </button>
              </div>
            </div>
          )}

          {step === 3 && (
            <div className="step-content fade-in">
              <h1>3. RRT-Connect</h1>
              <p className="subtitle">Bi-directional Search</p>

              <div className="explanation">
                <h3>Meeting in the Middle</h3>
                <p>
                  Standard RRT struggled with the wall. Now let's try
                  <b>RRT-Connect</b>.
                  <br />
                  <br />
                  It grows <b>two trees</b>: one from start, one from goal. They
                  aggressively try to meet in the middle.
                  <br />
                  <br />
                  <b>Action:</b> Even with the same "Weak" parameters, click
                  "Run". Does it solve it? Or does it get closer?
                  <br />
                  It's much more efficient, often finding a path where Standard
                  RRT fails completely.
                </p>

                {/* NEW: Comparison Dashboard */}
                {standardStats && (
                  <div
                    className="comparison-card"
                    style={{
                      marginTop: '1rem',
                      padding: '1rem',
                      background: 'rgba(0,0,0,0.2)',
                      borderRadius: '8px',
                    }}
                  >
                    <h4>Performance Comparison</h4>

                    <div
                      style={{
                        display: 'grid',
                        gridTemplateColumns: '1fr 1fr',
                        gap: '1rem',
                        marginBottom: '0.5rem',
                      }}
                    >
                      <div style={{ color: '#ff7777' }}>
                        <strong>Standard RRT</strong>
                        <br />
                        Time: {standardStats.time}ms
                        <br />
                        Nodes: {standardStats.nodes}
                      </div>

                      <div style={{ color: '#55ff55' }}>
                        <strong>RRT-Connect</strong>
                        <br />
                        {stats ? (
                          <>
                            Time: {stats.time}ms
                            <br />
                            Nodes: {stats.nodes}
                          </>
                        ) : (
                          <span>Running...</span>
                        )}
                      </div>
                    </div>

                    {stats && stats.success && standardStats.success && (
                      <div
                        style={{
                          borderTop: '1px solid rgba(255,255,255,0.1)',
                          paddingTop: '0.5rem',
                          textAlign: 'center',
                        }}
                      >
                        <strong>
                          Speedup:{' '}
                          {(
                            standardStats.time / Math.max(1, stats.time)
                          ).toFixed(1)}
                          x Faster
                        </strong>
                      </div>
                    )}

                    <p
                      style={{
                        fontSize: '0.8rem',
                        color: '#aaa',
                        marginTop: '0.5rem',
                      }}
                    >
                      The faint red ghost tree shows the volume Standard RRT had
                      to explore.
                    </p>
                  </div>
                )}
              </div>

              <div className="controls-section">
                <h3>Algorithm Parameters</h3>
                <div className="parameter-control">
                  <label>Step Size (Growth Rate): {rrtParams.stepSize}</label>
                  <input
                    type="range"
                    min="0.01"
                    max="0.5"
                    step="0.01"
                    value={rrtParams.stepSize}
                    onChange={(e) =>
                      handleParamChange('stepSize', parseFloat(e.target.value))
                    }
                  />
                </div>
                <div className="parameter-control">
                  <label>Max Iterations (Effort): {rrtParams.maxIter}</label>
                  <input
                    type="range"
                    min="1000"
                    max="10000"
                    step="1000"
                    value={rrtParams.maxIter}
                    onChange={(e) =>
                      handleParamChange('maxIter', parseInt(e.target.value))
                    }
                  />
                </div>
                <div className="parameter-control">
                  <label>Goal Bias: {rrtParams.goalBias}</label>
                  <input
                    type="range"
                    min="0"
                    max="1"
                    step="0.05"
                    value={rrtParams.goalBias}
                    onChange={(e) =>
                      handleParamChange('goalBias', parseFloat(e.target.value))
                    }
                  />
                </div>
              </div>

              <div className="nav-buttons">
                <button onClick={() => handleStepChange(2)}>Back</button>
                <button
                  className="run-btn"
                  onClick={() => controllerRef.current?.runPlanner()}
                  style={{
                    backgroundColor: '#2ecc71',
                    color: 'white',
                    fontWeight: 'bold',
                  }}
                >
                  ▶ Run Planner
                </button>
              </div>
            </div>
          )}
        </div>

        {/* Right Panel: The Engineer's Dashboard (Persistent) */}
        <div className="dashboard-panel">
          <h3>Simulation Control</h3>

          <div className="control-group">
            <h4>Scenario</h4>
            <div className="button-group">
              <button
                className={scenario === 'easy' ? 'active' : ''}
                onClick={() => handleScenarioChange('easy')}
              >
                Free
              </button>
              <button
                className={scenario === 'medium' ? 'active' : ''}
                onClick={() => handleScenarioChange('medium')}
              >
                Wall
              </button>
              <button
                className={scenario === 'hard' ? 'active' : ''}
                onClick={() => handleScenarioChange('hard')}
              >
                Deep
              </button>
            </div>
          </div>

          {targetPos && (
            <div className="stat-card" style={{ marginBottom: '1rem' }}>
              <div className="stat-row">
                <span className="label">Target X</span>
                <span className="value">{targetPos.x.toFixed(2)}</span>
              </div>
              <div className="stat-row">
                <span className="label">Target Y</span>
                <span className="value">{targetPos.y.toFixed(2)}</span>
              </div>
              <div className="stat-row">
                <span className="label">Target Z</span>
                <span className="value">{targetPos.z.toFixed(2)}</span>
              </div>
            </div>
          )}

          <div className="control-group">
            <h4>Obstacle Height</h4>
            <div className="parameter-control">
              <input
                type="range"
                min="0.1"
                max="4.0"
                step="0.1"
                value={obstacleHeight}
                onChange={(e) =>
                  handleObstacleHeightChange(parseFloat(e.target.value))
                }
              />
              <div
                style={{
                  fontSize: '0.7rem',
                  color: '#888',
                  textAlign: 'right',
                }}
              >
                {obstacleHeight.toFixed(1)}m
              </div>
            </div>
          </div>

          <div className="control-group">
            <h4>Robot</h4>
            <div className="button-group">
              <button
                onClick={() => controllerRef.current?.addJoint()}
                style={
                  step === 2 && (!stats || !stats.success)
                    ? {
                        border: '1px solid #f1c40f',
                        color: '#f1c40f',
                        boxShadow: '0 0 5px rgba(241, 196, 15, 0.3)',
                      }
                    : {}
                }
              >
                + Joint
              </button>
              <button onClick={() => controllerRef.current?.removeJoint()}>
                - Joint
              </button>
              <button onClick={() => controllerRef.current?.resetJoints()}>
                Reset
              </button>
            </div>
          </div>

          {step >= 2 && stats && (
            <div className="stat-card fade-in">
              <div className="stat-row">
                <span className="label">Status</span>
                <span
                  className={`value ${stats.success ? 'success' : 'pending'}`}
                >
                  {stats.success ? 'CONVERGED' : 'PLANNING'}
                </span>
              </div>
              <div className="stat-row">
                <span className="label">Time</span>
                <span className="value">{stats.time} ms</span>
              </div>
              <div className="stat-row">
                <span className="label">Nodes</span>
                <span className="value">{stats.nodes}</span>
              </div>
            </div>
          )}

          <div className="debug-section">
            <label className="toggle-row">
              <input
                type="checkbox"
                checked={debugMode}
                onChange={toggleDebug}
              />
              <span>Debug Hitboxes</span>
            </label>
          </div>
        </div>
      </div>

      <div className="controls-help fade-in">
        <span>
          <span className="key">LMB</span> Rotate
        </span>
        <span>
          <span className="key">RMB</span> Pan
        </span>
        <span>
          <span className="key">Scroll</span> Zoom
        </span>
      </div>
    </div>
  )
}

export default App
