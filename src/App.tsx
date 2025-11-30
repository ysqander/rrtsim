import { useEffect, useRef, useState } from 'react'
import { SceneController, type PlannerStats } from './logic/SceneController'
import './App.css'

function App() {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const controllerRef = useRef<SceneController | null>(null)
  const [stats, setStats] = useState<PlannerStats | null>(null)
  const [scenario, setScenario] = useState<'easy' | 'medium' | 'hard'>('medium')
  const [debugMode, setDebugMode] = useState(false)

  // Obstacle Height (Default 2.0)
  const [obstacleHeight, setObstacleHeight] = useState(2.0)

  // 0: Intro, 1: Greedy IK, 2: RRT-Connect
  const [step, setStep] = useState<0 | 1 | 2>(0)

  // RRT Parameters
  const [rrtParams, setRrtParams] = useState({
    stepSize: 0.05,
    maxIter: 5000,
    goalBias: 0.05,
  })

  // 1. Initialize Scene
  useEffect(() => {
    if (!canvasRef.current) return
    const controller = new SceneController(canvasRef.current)
    controllerRef.current = controller
    controller.onStatsUpdate = setStats

    // Initial Scenario
    controller.setScenario(scenario)
    controller.setAlgorithm('rrt') // Default background mode

    const onResize = () =>
      controller.resize(window.innerWidth, window.innerHeight)
    window.addEventListener('resize', onResize)

    return () => window.removeEventListener('resize', onResize)
  }, []) // One-time init, ignore scenario dependency as it's handled by state setter

  // 2. React to Step Changes
  const updateAlgorithmForStep = (newStep: number) => {
    if (!controllerRef.current) return

    if (newStep === 1) {
      // Switching to Greedy: We can reset or keep.
      // Usually entering Step 1 implies a fresh start or continue from Intro.
      // Let's reset robot to home to avoid confusion, but keep scenario.
      controllerRef.current.setAlgorithm('greedy', true)
    } else if (newStep === 2) {
      // Switching to RRT: THIS IS THE KEY CHANGE.
      // We do NOT reset the robot position if we are coming from Greedy.
      // Actually, we WANT to reset the robot to home (safe start) but KEEP the target in the hard spot.
      // `setAlgorithm('rrt', true)` resets robot to home, keeps target.
      controllerRef.current.setAlgorithm('rrt', true)
      // Force update params to ensure they are applied
      controllerRef.current.updateRRTParams(rrtParams)
    } else {
      // Intro
      controllerRef.current.setAlgorithm('rrt')
    }
  }

  // 3. React to Param Changes
  useEffect(() => {
    if (controllerRef.current && step === 2) {
      controllerRef.current.updateRRTParams(rrtParams)
    }
  }, [rrtParams, step])

  // Handlers
  const handleStepChange = (newStep: 0 | 1 | 2) => {
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
              2. RRT-Connect
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
              </div>

              <div className="nav-buttons">
                <button onClick={() => handleStepChange(0)}>Back</button>
                <button
                  className="primary-btn"
                  onClick={() => handleStepChange(2)}
                >
                  Next: The Solution &rarr;
                </button>
              </div>
            </div>
          )}

          {step === 2 && (
            <div className="step-content fade-in">
              <h1>2. RRT-Connect</h1>
              <p className="subtitle">Rapidly-exploring Random Trees</p>

              <div className="explanation">
                <h3>Intuition: Probabilistic Planning</h3>
                <p className="placeholder-text">
                  [PLACEHOLDER: RRT INTUITION]
                  <br />
                  Instead of greedily moving to the goal, we grow a "Tree" of
                  possibilities. We randomly sample points in space and try to
                  connect them.
                  <br />
                  <br />
                  RRT-Connect grows two trees: one from the start, one from the
                  goal. They aggressively try to meet in the middle. This allows
                  the robot to explore the empty space and find a path *around*
                  obstacles, rather than trying to force its way through.
                </p>
              </div>

              <div className="controls-section">
                <h3>Algorithm Parameters</h3>
                <div className="parameter-control">
                  <label>Step Size (Growth Rate): {rrtParams.stepSize}</label>
                  <input
                    type="range"
                    min="0.01"
                    max="0.2"
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
              <button onClick={() => controllerRef.current?.addJoint()}>
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

          {step === 2 && stats && (
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
    </div>
  )
}

export default App
