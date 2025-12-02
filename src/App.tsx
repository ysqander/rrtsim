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
  // Tutorial Mode: If true, enforces lesson scenarios. If false, allows free exploration.
  const [tutorialMode, setTutorialMode] = useState(true)
  // Auto-Swivel: Automatically orbit camera after running planner (once, then auto-disables)
  const [autoSwivel, setAutoSwivel] = useState(true)
  // Show comparison with Standard RRT in step 3 (requires running Std RRT first in step 2)
  const [showComparison, setShowComparison] = useState(true)

  // Obstacle Height (Default 2.0)
  const [obstacleHeight, setObstacleHeight] = useState(2.0)

  // Joint Count (displayed next to Robot section)
  const [jointCount, setJointCount] = useState(4) // Default robot has 4 joints

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

    // Get initial joint count
    setJointCount(controller.getJointCount())

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

      if (tutorialMode) {
        // HARDCODED FAIL CASE:
        // Wall is at z=1. Robot at z=0.
        // Target at (0.5, 1.5, 2.0) -> Behind wall.
        // Robot tries to go straight -> Collision.
        controllerRef.current.setTargetPosition(0.39, 1.65, 2.0)
      }
    } else if (newStep === 2) {
      // Switching to Standard RRT (The "Bushy" Tree)
      console.log('[DEBUG] Entering Step 2. TutorialMode:', tutorialMode)
      controllerRef.current.setAlgorithm('rrt-standard', true)
      console.log('[DEBUG] setAlgorithm done')
      controllerRef.current.setInteraction(true)

      // Zoom out slightly to see the whole robot
      controllerRef.current.camera.position.set(4, 4, 8)
      controllerRef.current.camera.lookAt(0, 1, 0)

      if (tutorialMode) {
        console.log('[DEBUG] Tutorial mode: resetting scene')
        // FORCE RESET SCENE STATE for consistent "Failure" Demo
        // 1. Reset Joints
        controllerRef.current.resetJoints()
        setJointCount(controllerRef.current.getJointCount())
        console.log('[DEBUG] resetJoints done')
        // 2. Reset Obstacle Height
        setObstacleHeight(2.0)
        controllerRef.current.setObstacleHeight(2.0)
        console.log('[DEBUG] setObstacleHeight done')
        // 3. Reset Target Position to the "Behind Wall" spot
        controllerRef.current.setTargetPosition(2.24, 2.59, 2.0)
        console.log('[DEBUG] setTargetPosition done')
      }

      // Apply WEAK defaults for "Failure" demo
      // We might want to apply these even in non-tutorial mode for comparison,
      // OR maybe we should trust the user's settings?
      // Let's stick to the plan: "Parameter defaults should probably still happen"
      // but if it's NOT tutorial mode, maybe we shouldn't force them?
      // The prompt implies "free exploration", so let's ONLY set weak params in Tutorial Mode.
      if (tutorialMode) {
        const weakParams = { stepSize: 0.05, maxIter: 5000, goalBias: 0.0 }
        setRrtParams(weakParams)
        controllerRef.current.updateRRTParams(weakParams)
        console.log('[DEBUG] updateRRTParams done')
      }
    } else if (newStep === 3) {
      // Switching to RRT-Connect (The "Fast" Solution)
      controllerRef.current.setAlgorithm('rrt', true)
      controllerRef.current.setInteraction(true)

      // Respect the comparison toggle for ghost tree visibility
      controllerRef.current.setGhostTreeVisible(showComparison)

      if (tutorialMode) {
        // Apply WEAK defaults again to show improvement
        const weakParams = { stepSize: 0.05, maxIter: 5000, goalBias: 0.0 }
        setRrtParams(weakParams)
        controllerRef.current.updateRRTParams(weakParams)
      }
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

  const toggleTutorialMode = () => {
    setTutorialMode(!tutorialMode)
  }

  const handleParamChange = (key: keyof typeof rrtParams, value: number) => {
    setRrtParams((prev) => ({ ...prev, [key]: value }))
  }

  // Gating helpers
  const isInteractiveStep = step >= 1

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
              <span className="step-number">0</span>
              <span className="step-label">Intro</span>
            </button>
            <button
              className={step === 1 ? 'active' : ''}
              onClick={() => handleStepChange(1)}
            >
              <span className="step-number">1</span>
              <span className="step-label">Greedy</span>
            </button>
            <button
              className={step === 2 ? 'active' : ''}
              onClick={() => handleStepChange(2)}
            >
              <span className="step-number">2</span>
              <span className="step-label">Std RRT</span>
            </button>
            <button
              className={step === 3 ? 'active' : ''}
              onClick={() => handleStepChange(3)}
            >
              <span className="step-number">3</span>
              <span className="step-label">RRT Connect</span>
            </button>
          </div>

          {step === 0 && (
            <div className="step-content fade-in">
              <h1>Robotic Motion Planning</h1>
              <p className="subtitle">An Interactive Introduction</p>

              <div className="explanation">
                <h3>Welcome</h3>
                <p className="placeholder-text">
                  <br />
                  Robotics is about movement under constraints. But how do we
                  get from A to B without hitting obstacle C? In this
                  interactive tutorial, we will explore an advanced algorithm
                  used in industrial applications that help us plan paths calls
                  RRT-connect.
                  <br />
                  <br />
                  You don't need to know anything about robotics to learn this.
                  This is a tutorial to build intuition using a multiple joint
                  robot arm. The constraints are the number of joints, the
                  angles that the joints can take and the length of the arms.
                  We'll start with a simplistic approach "The Greedy Approach"
                  and see a case where it fails. Then we'll explore RRT or
                  Rapidly-exploring Random Tree and its more efficient variant
                  RRT-Connect)
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
              <div className="explanation">
                <p className="placeholder-text">
                  <br />
                  This uses Cyclic Coordinate Descent algorithm. It is as if
                  each joint in the robot arm, "looks" at the current position
                  of the tip of the robot and the target position and decides to
                  rotate itself to minimize that distance.
                  <br />
                  <br />
                  By looping through each joint, and doing this wiggle rotation
                  adjustment at each joint, we get closer and closer. In low
                  complexity cases, the end point reaches the target.
                </p>

                <h3 style={{ marginTop: '2rem', fontSize: '1.25rem' }}>
                  Why it Fails
                </h3>
                <p className="placeholder-text">
                  Greedy algorithms optimize for the <strong>IMMEDIATE</strong>{' '}
                  shortest path. Each joint rotates to get the tip closer to the
                  target right now. It doesn't "plan ahead" or realize that
                  sometimes you need to move *away* from the target (or around a
                  wall) to eventually reach it.
                </p>
              </div>

              <div className="callout-box notice">
                <strong>What to Notice</strong>
                <p>
                  Watch how the robot moves directly toward the target. When the
                  wall is in the way, it gets stuck - it can't "plan ahead" to
                  go around.
                </p>
              </div>

              <div className="controls-section">
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

              <div className="callout-box action">
                <strong>Try This Next</strong>
                <p>
                  Drag the target to different positions. Can you find spots
                  where Greedy succeeds vs fails?
                </p>
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
              <p className="subtitle">Single Tree Random Exploration</p>

              <div className="explanation">
                <p>
                  This algorithm grows a "Tree" of valid movements—essentially a
                  map of safe places the robot has visited. It starts by picking
                  a random joint configuration somewhere in the room. Then, it
                  looks at its existing map (the tree) and finds the spot
                  closest to that random point. From there, it grows a single
                  small "twig" toward the random target. If this new step
                  doesn't hit a wall, it's added to the map. In the next
                  iteration, it picks a brand new random target and repeats the
                  process, slowly building a bushy web of safe paths until one
                  of them touches the goal.
                  <br />
                  <br />
                  <b>Try it:</b> Click "Run Planner".
                  <br />
                  Notice how "bushy" the tree is? It explores everywhere! But
                  for this "Behind the Wall" target, it will likely{' '}
                  <b>
                    FAIL (the robot tip will not move to the target position)
                  </b>{' '}
                  or time out. This is because this algorithm search a little
                  all over the place (since it picks points to extend toward
                  randomly).
                  <br />
                  <br />
                  <b>Parameter Guide:</b>
                  <br />- <b>Step Size:</b> How far the robot reaches in each
                  step. Larger steps jump gaps but might hit walls.
                  <br />- <b>Iterations:</b> How many attempts to make. More
                  attempts = higher chance of success but slower.
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

              <div className="callout-box notice">
                <strong>What to Notice</strong>
                <p>
                  See how the tree grows in all directions? This "bushy"
                  exploration is thorough but slow. With weak parameters, it
                  often times out.
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
              </div>

              <div className="callout-box action">
                <strong>Try This Next</strong>
                <p>
                  Increase Step Size and Max Iterations. Can you get it to find
                  the target? Notice how many nodes it needs.
                </p>
              </div>

              <div
                className="nav-buttons"
                style={{ flexDirection: 'column', gap: '0.5rem' }}
              >
                <button
                  className={`primary-btn ${!stats ? 'pulse-animation' : ''}`}
                  onClick={() => {
                    controllerRef.current?.runPlanner()
                    if (autoSwivel) {
                      // Trigger the "Swivel" animation to show off the scene
                      setTimeout(() => {
                        controllerRef.current?.animateCameraOutro()
                      }, 1000) // Wait 1s for tree to start appearing
                      // Auto-disable after first use
                      setAutoSwivel(false)
                    }
                  }}
                  style={{ width: '100%', marginBottom: '0.5rem' }}
                >
                  ▶ Run Planner
                </button>
                <div
                  style={{
                    display: 'flex',
                    gap: '1rem',
                    justifyContent: 'space-between',
                  }}
                >
                  <button
                    onClick={() => handleStepChange(1)}
                    style={{ flex: 1 }}
                  >
                    Back
                  </button>
                  <button
                    onClick={() => handleStepChange(3)}
                    disabled={!stats}
                    style={{
                      flex: 1,
                      border: '1px solid rgba(255,255,255,0.2)',
                      opacity: stats ? 1 : 0.3,
                      cursor: stats ? 'pointer' : 'not-allowed',
                    }}
                  >
                    Next: Optimization
                  </button>
                </div>
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
                  Standard RRT might struggle to find the target if it
                  relatively far and in a complex position (behind obstacle).
                  It's also inneficient. Bi directional RRT connect is a
                  solution.
                  <br />
                  <br />
                  The algo grows <b>two trees</b>: one from start, one from
                  goal. They aggressively try to meet in the middle.
                  <br />
                  <br />
                  Each tree takes turn. One takes a step toward a random point
                  and the other one "looks" at where the new extension is and
                  tried to extend its nearest node to it. then the swap roles
                  and redo the same process.
                  <br />
                  <br />
                  <b>
                    This random step and connect in turn might seem like it'S
                    going to be chaotic but it is actually very efficient.
                  </b>
                  <br />
                  <br />
                  It's much more efficient, often finding a path where Standard
                  RRT fails completely.
                </p>

                <div className="callout-box action">
                  <strong>Try This Next</strong>
                  <p>
                    Even with the same "Weak" parameters, click "Run". Does it
                    solve it? Or does it get closer?
                  </p>
                </div>

                {/* Comparison Toggle */}
                <label
                  className="toggle-row"
                  style={{ marginTop: '1rem', marginBottom: '0.5rem' }}
                >
                  <input
                    type="checkbox"
                    checked={showComparison}
                    onChange={(e) => {
                      setShowComparison(e.target.checked)
                      controllerRef.current?.setGhostTreeVisible(
                        e.target.checked
                      )
                    }}
                  />
                  <span style={{ color: showComparison ? '#2ecc71' : '#aaa' }}>
                    Show Comparison with Std RRT
                  </span>
                </label>

                {/* Comparison Dashboard */}
                {showComparison && standardStats && (
                  <div
                    className="comparison-card"
                    style={{
                      marginTop: '0.5rem',
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
                      to explore (on the last attempt you ran on tab 2. Std
                      RRT).
                    </p>
                  </div>
                )}

                {/* Hint when no comparison data */}
                {showComparison && !standardStats && (
                  <p
                    style={{
                      fontSize: '0.85rem',
                      color: '#f1c40f',
                      marginTop: '0.5rem',
                      fontStyle: 'italic',
                    }}
                  >
                    To see a comparison, first run Standard RRT in step 2, then
                    return here.
                  </p>
                )}
              </div>

              <div className="callout-box notice">
                <strong>What to Notice</strong>
                <p>
                  Two trees grow toward each other - much more directed! Compare
                  the node count and time to Standard RRT.
                </p>
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
                <p className="tiny-text" style={{ marginBottom: '0.3rem' }}>
                  Goal Bias: How often it tries to go straight to the goal.
                  Higher bias = faster but might get stuck.
                </p>
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

          {/* Step 0 Gating Message */}
          {!isInteractiveStep && (
            <p className="gated-hint">Start the tutorial to unlock controls</p>
          )}

          {/* Visual Settings Group */}
          <div
            className={`control-group ${
              !isInteractiveStep ? 'gated-control' : ''
            }`}
          >
            <h4>Visual Settings</h4>

            <label className="toggle-row" style={{ marginBottom: '0.5rem' }}>
              <input
                type="checkbox"
                checked={tutorialMode}
                onChange={toggleTutorialMode}
              />
              <span style={{ color: tutorialMode ? '#2ecc71' : '#aaa' }}>
                Tutorial Mode {tutorialMode ? '(ON)' : '(OFF)'}
              </span>
            </label>
            <p className="tiny-text" style={{ marginBottom: '0.8rem' }}>
              Resets scene to pre-configured states for each lesson step.
            </p>

            <label className="toggle-row" style={{ marginBottom: '0.5rem' }}>
              <input
                type="checkbox"
                checked={autoSwivel}
                onChange={() => setAutoSwivel(!autoSwivel)}
              />
              <span style={{ color: autoSwivel ? '#2ecc71' : '#aaa' }}>
                Auto-Swivel Camera
              </span>
            </label>
            <p className="tiny-text" style={{ marginBottom: '0.8rem' }}>
              Orbits camera after running planner (auto-disables after first
              use).
            </p>

            <button
              onClick={() => controllerRef.current?.setTargetPosition(2, 2, 0)}
              style={{ width: '100%', marginBottom: '0.5rem' }}
            >
              Reset Target Position
            </button>
            <button
              onClick={() => controllerRef.current?.resetRobotPosition()}
              style={{ width: '100%', marginBottom: '0.5rem' }}
            >
              Reset Robot Position
            </button>
          </div>

          <div
            className={`control-group ${
              !isInteractiveStep ? 'gated-control' : ''
            }`}
          >
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

          <div
            className={`control-group ${
              !isInteractiveStep ? 'gated-control' : ''
            }`}
          >
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

          <div
            className={`control-group ${
              !isInteractiveStep ? 'gated-control' : ''
            }`}
          >
            <h4>
              Robot{' '}
              <span
                style={{
                  fontSize: '0.85rem',
                  color: '#888',
                  fontWeight: 'normal',
                }}
              >
                ({jointCount} joints)
              </span>
            </h4>
            <div className="button-group">
              <button
                onClick={() => {
                  controllerRef.current?.addJoint()
                  if (controllerRef.current) {
                    setJointCount(controllerRef.current.getJointCount())
                  }
                }}
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
              <button
                onClick={() => {
                  controllerRef.current?.removeJoint()
                  if (controllerRef.current) {
                    setJointCount(controllerRef.current.getJointCount())
                  }
                }}
              >
                - Joint
              </button>
              <button
                onClick={() => {
                  controllerRef.current?.resetJoints()
                  if (controllerRef.current) {
                    setJointCount(controllerRef.current.getJointCount())
                  }
                }}
              >
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
