import { useEffect, useRef, useState } from 'react'
import { SceneController, type PlannerStats } from './logic/SceneController'
import type { FailureReason } from './logic/RRTPlanner'
import './App.css'

// Maps failure reasons to user-friendly messages and suggestions
function getFailureFeedback(
  reason: FailureReason,
  isGreedy?: boolean,
  step?: number
): { message: string; suggestions: string[] } | null {
  if (!reason) return null

  // Greedy (CCD) has no tunable parameters - just suggest moving the target
  if (isGreedy) {
    const messages: Record<string, string> = {
      goal_in_collision: 'Direct path blocked by obstacle',
      self_collision: 'Robot would collide with itself',
      unreachable: 'Target appears unreachable',
      timeout: 'Could not reach target',
    }
    return {
      message: messages[reason] || 'Could not reach target',
      suggestions: ['Move target to another position'],
    }
  }

  switch (reason) {
    case 'timeout': {
      // Base suggestions for timeout
      const suggestions = [
        'Increase Max Iterations',
        'Increase Step Size for faster exploration',
      ]
      // Only show Goal Bias suggestion in Step 3 where it's visible
      if (step !== 2) {
        suggestions.push('Increase Goal Bias to focus search')
      }
      return {
        message: 'Iteration budget exhausted',
        suggestions,
      }
    }
    case 'unreachable':
      return {
        message: 'Target appears unreachable',
        suggestions: [
          'Move target closer to robot base',
          'Add more joints for extended reach',
        ],
      }
    case 'goal_in_collision':
      return {
        message: 'Target position inside obstacle',
        suggestions: [
          'Move target away from the obstacle',
          'Lower the obstacle height',
        ],
      }
    case 'self_collision':
      return {
        message: 'Path causes robot self-collision',
        suggestions: [
          'Move target to a less constrained position',
          'Remove joints for simpler arm geometry',
        ],
      }
    default:
      return null
  }
}

// Parameter presets for quick tuning
const RRT_PRESETS = {
  weak: {
    stepSize: 0.05,
    maxIter: 5000,
    goalBias: 0.0,
    seed: 40,
    label: 'Weak',
    description: 'Tutorial defaults - likely to fail',
  },
  balanced: {
    stepSize: 0.2,
    maxIter: 10000,
    goalBias: 0.15,
    seed: 40,
    label: 'Balanced',
    description: 'Good balance of speed and thoroughness',
  },
  strong: {
    stepSize: 0.3,
    maxIter: 15000,
    goalBias: 0.2,
    seed: 40,
    label: 'Strong',
    description: 'Higher success rate for hard problems',
  },
} as const

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
  // Tutorial Mode: If true, enforces lesson scenarios. If false, allows free exploration.
  const [tutorialMode, setTutorialMode] = useState(true)
  // Auto-Swivel: Automatically orbit camera after running planner (once, then auto-disables)
  const [autoSwivel, setAutoSwivel] = useState(true)
  // Show comparison with Standard RRT in step 3 (requires running Std RRT first in step 2)
  const [showComparison, setShowComparison] = useState(true)
  // Show/hide RRT trees in steps 2 and 3
  const [showTree, setShowTree] = useState(true)

  // Comparison mode progress indicator (for Step 3 tutorial mode)
  const [comparisonPhase, setComparisonPhase] = useState<
    'idle' | 'standard' | 'connect' | 'done'
  >('idle')

  // Obstacle Preset System
  const [obstaclePreset, setObstaclePreset] = useState<
    'wall' | 'inverted_u' | 'corridor'
  >('wall')
  const [obstaclePos, setObstaclePos] = useState({ x: 1.0, z: 0 })
  const [obstacleRotation, setObstacleRotation] = useState(0) // degrees, 0-360
  const [obstacleTransformMode, setObstacleTransformMode] = useState<
    'translate' | 'rotate'
  >('translate')
  const [editingTarget, setEditingTarget] = useState(true) // true = target, false = obstacles

  // Shape-specific dimension state
  const [wallDims, setWallDims] = useState({
    width: 3.0,
    height: 3.0,
    thickness: 0.2,
  })
  const [gateDims, setGateDims] = useState({
    gapWidth: 1.6,
    height: 3.2,
    pillarThickness: 0.4,
  })
  const [corridorDims, setCorridorDims] = useState({
    corridorWidth: 0.6,
    length: 1.5,
    height: 1.5,
  })

  // Joint Count (displayed next to Robot section)
  const [jointCount, setJointCount] = useState(9) // Default robot has 9 joints

  // Arm Length Scale (0.7 = default for thinner robot)
  const [armLength, setArmLength] = useState(0.7)

  // Joint Width Scale (0.3 = default for thin robot)
  const [jointWidth, setJointWidth] = useState(0.3)

  // Tip (End Effector) Size Scale (0.6 = default to match joint size)
  const [tipSize, setTipSize] = useState(0.6)

  // Camera Settings
  const [cameraZoom, setCameraZoom] = useState(10)
  const [cameraAzimuth, setCameraAzimuth] = useState(0)
  const [cameraPolar, setCameraPolar] = useState(45)

  // 0: Intro, 1: Greedy IK, 2: Standard RRT, 3: RRT-Connect
  const [step, setStep] = useState<0 | 1 | 2 | 3>(0)

  // RRT Parameters
  // Initial params match the "Weak" preset (tutorial mode default)
  const [rrtParams, setRrtParams] = useState({
    stepSize: 0.05,
    maxIter: 5000,
    goalBias: 0.0,
    seed: 40,
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
    controller.onObstacleMove = (pos) => setObstaclePos(pos)
    controller.onObstacleRotate = (rotation) => setObstacleRotation(rotation)
    controller.onCameraChange = (data) => {
      setCameraZoom(data.zoom)
      setCameraAzimuth(data.azimuth)
      setCameraPolar(data.polar)
    }

    // Comparison mode callbacks (for Step 3 fair comparison)
    controller.onComparisonProgress = (phase) => setComparisonPhase(phase)
    controller.onStandardStatsUpdate = (stdStats) => setStandardStats(stdStats)

    // Get initial joint count
    setJointCount(controller.getJointCount())

    // Set initial joint width (default is 0.6, thinner than original)
    controller.setJointWidth(jointWidth)

    // Set initial tip size (default is 1.0)
    controller.setTipSize(tipSize)

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

      if (tutorialMode) {
        // Reset joints to default, then reduce to 4 joints for simplicity
        controllerRef.current.resetJoints()
        // Remove joints until we have 4 (default is 9, so remove 5)
        for (let i = 0; i < 5; i++) {
          controllerRef.current.removeJoint()
        }
        controllerRef.current.resetRobotPosition()
        setJointCount(controllerRef.current.getJointCount())

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

      // Set default camera view for Standard RRT: Zoom: 12.6, Azimuth: 347°, Polar: 63°
      controllerRef.current.controls.target.set(0, 1, 0)
      controllerRef.current.setCameraZoom(12.6)
      controllerRef.current.setCameraAngle(347, 63)

      // Update React state to match these defaults (syncs UI sliders)
      setCameraZoom(12.6)
      setCameraAzimuth(347)
      setCameraPolar(63)

      if (tutorialMode) {
        console.log('[DEBUG] Tutorial mode: resetting scene')
        // FORCE RESET SCENE STATE for consistent "Failure" Demo
        // 1. Reset Joints (rebuilds robot with default config)
        controllerRef.current.resetJoints()
        setJointCount(controllerRef.current.getJointCount())
        // 1b. Explicitly reset robot to home position (all angles = 0)
        controllerRef.current.resetRobotPosition()
        console.log('[DEBUG] resetJoints + resetRobotPosition done')
        // 2. Reset Obstacle to wall preset
        setObstaclePreset('wall')
        controllerRef.current.setObstaclePreset('wall', wallDims)
        setEditingTarget(true)
        controllerRef.current.setEditingTarget(true)
        console.log('[DEBUG] setObstaclePreset done')
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
        const weakParams = {
          stepSize: 0.05,
          maxIter: 5000,
          goalBias: 0.0,
          seed: 40,
        }
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
        // Setup a gate environment for RRT-Connect demo
        // This showcases how both trees grow toward each other through the gap

        // Reset joints to default 9-joint configuration
        controllerRef.current.resetJoints()
        setJointCount(controllerRef.current.getJointCount())

        // Use inverted_u (gate) preset
        const gateDefaults = {
          gapWidth: 1.6,
          height: 3.2,
          pillarThickness: 0.4,
        }
        controllerRef.current.setObstaclePreset('inverted_u', gateDefaults)
        setObstaclePreset('inverted_u')
        setGateDims(gateDefaults)

        // Set obstacle position and rotation
        controllerRef.current.setObstaclePosition(3.11, 1.15)
        controllerRef.current.setObstacleRotation(321)
        setObstaclePos({ x: 3.11, z: 1.15 })
        setObstacleRotation(321)

        setEditingTarget(true)
        controllerRef.current.setEditingTarget(true)

        // Reset robot to home position
        controllerRef.current.resetRobotPosition()

        // Target positioned behind the gate
        controllerRef.current.setTargetPosition(3.31, 1.53, 1.88)

        // Apply balanced defaults that find a neat solution
        const balancedParams = {
          stepSize: 0.2,
          maxIter: 10000,
          goalBias: 0.15,
          seed: 40,
        }
        setRrtParams(balancedParams)
        controllerRef.current.updateRRTParams(balancedParams)

        // Set default camera view for RRT-Connect: Zoom: 11.5, Azimuth: 32°, Polar: 67°
        controllerRef.current.controls.target.set(0, 1, 0)
        controllerRef.current.setCameraZoom(11.5)
        controllerRef.current.setCameraAngle(32, 67)

        // Update React state to match these defaults (syncs UI sliders)
        setCameraZoom(11.5)
        setCameraAzimuth(32)
        setCameraPolar(67)
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
    // This ensures we compare against the EXACT run the user just saw (non-tutorial mode).
    if (step === 2 && stats && !tutorialMode) {
      setStandardStats(stats)
    }

    // Reset comparison phase when entering step 3 or leaving it
    if (newStep === 3 || step === 3) {
      setComparisonPhase('idle')
    }

    // Clear standardStats when entering step 3 in tutorial mode
    // (will be populated by the comparison run)
    if (newStep === 3 && tutorialMode) {
      setStandardStats(null)
    }

    setStep(newStep)
    updateAlgorithmForStep(newStep)
  }

  const handleScenarioChange = (type: 'easy' | 'medium' | 'hard') => {
    setScenario(type)
    controllerRef.current?.setScenario(type)
    // Reset editing mode to target when changing scenarios
    setEditingTarget(true)
    controllerRef.current?.setEditingTarget(true)
  }

  const handleResetObstacle = () => {
    if (!controllerRef.current) return

    if (step === 3) {
      // Step 3: Gate (Inverted U)
      const defaultGateDims = {
        gapWidth: 1.6,
        height: 3.2,
        pillarThickness: 0.4,
      }

      // Update Controller
      controllerRef.current.setObstaclePreset('inverted_u', defaultGateDims)
      controllerRef.current.setObstaclePosition(3.11, 1.15)
      controllerRef.current.setObstacleRotation(321)

      // Update State
      setObstaclePreset('inverted_u')
      setGateDims(defaultGateDims)
      setObstaclePos({ x: 3.11, z: 1.15 })
      setObstacleRotation(321)
    } else {
      // Step 2 (and others): Wall
      const defaultWallDims = {
        width: 3.0,
        height: 3.0,
        thickness: 0.2,
      }

      // Update Controller
      // Note: setObstaclePreset resets position to default (1.0, 0) if preservePosition is false
      controllerRef.current.setObstaclePreset('wall', defaultWallDims)
      controllerRef.current.setObstacleRotation(0)

      // Update State
      setObstaclePreset('wall')
      setWallDims(defaultWallDims)
      setObstaclePos({ x: 1.0, z: 0 })
      setObstacleRotation(0)
    }

    // Reset transform mode to translate for consistency
    setObstacleTransformMode('translate')
    controllerRef.current.setObstacleTransformMode('translate')
  }

  const toggleTutorialMode = () => {
    setTutorialMode(!tutorialMode)
  }

  const handleParamChange = (key: keyof typeof rrtParams, value: number) => {
    setRrtParams((prev) => ({ ...prev, [key]: value }))
  }

  // Gating helpers
  const isInteractiveStep = step >= 1
  const isObstacleEditable = step >= 2 // Obstacle controls only useful in RRT steps

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
              <div className="explanation">
                <p className="placeholder-text">
                  Robotics is about movement under constraints. But how do we
                  get from A to B without hitting obstacle C? In this
                  interactive tutorial, we will explore an advanced algorithm
                  used in industrial applications that help us plan paths calls
                  RRT-connect.
                  <br />
                  <br />
                  You don't need to know anything about robotics. This is a
                  tutorial to build intuition using a multiple joint robot arm.
                  The constraints are the number of joints, the angles that the
                  joints can take and the length of the arms. <br />
                  <br /> We'll start with a simplistic approach "The Greedy
                  Approach" and see a case where it fails. Then we'll explore
                  standard RRT (Rapidly-exploring Random Tree) and its more
                  efficient variant RRT-Connect
                </p>
              </div>

              <button
                className="primary-btn"
                onClick={() => handleStepChange(1)}
              >
                Start &rarr;
              </button>
            </div>
          )}

          {step === 1 && (
            <div className="step-content fade-in">
              <h1>1. The Greedy Approach</h1>
              <div className="explanation">
                <p className="placeholder-text">
                  This uses the Cyclic Coordinate Descent algorithm. It is as if
                  each joint in the robot arm "looks" at the current position of
                  the tip of the robot and the target position where we want the
                  tip to go and decides to rotate itself such that the distance
                  between the two points is minimized.
                  <br />
                  <br />
                  By looping through each joint, and doing this wiggle rotation
                  adjustment at each joint, we get closer and closer. In low
                  complexity cases, the endpoint reaches the target.
                </p>
                <br />
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
                <br />
                <div className="callout-box notice">
                  <strong>What to Notice</strong>
                  <p>
                    Watch how the robot moves directly toward the target. Note:
                    the robot turns red when it is beyond a safety margin of an
                    obstacle.
                  </p>
                </div>
                <br />
                <div className="callout-box action">
                  <strong>Try This Next</strong>
                  <p>
                    Drag the target to different positions. Can you find spots
                    where Greedy succeeds vs fails?
                  </p>
                </div>

                <h3 style={{ marginTop: '2rem', fontSize: '1.25rem' }}>
                  Why Greedy Fails often
                </h3>
                <p className="placeholder-text">
                  Greedy algorithms optimize for the <strong>IMMEDIATE</strong>{' '}
                  shortest path. Each joint rotates to get the tip closer to the
                  target right now. It doesn't "plan ahead" or realize that
                  sometimes you need to move *away* from the target (or around a
                  wall) to eventually reach it.
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
              <h1>2. Standard RRT (Rapidly exploring Random Tree)</h1>

              <div className="explanation">
                <p>
                  This algorithm grows a tree of valid movements: essentially a
                  map of safe places the robot has visited. It starts by picking
                  a random joint configuration somewhere in the room. Then, it
                  looks at its existing map (the tree) and finds the spot
                  closest to that random point. From there, it grows a single
                  small "twig" toward the random target. <br /> <br /> If this
                  new step doesn't hit a wall, it's added to the tree. In the
                  next iteration, it picks a brand new random target and repeats
                  the process, slowly building a bushy web of safe paths until
                  one of them touches the goal.
                  <br />
                  <br />
                  <div className="callout-box action">
                    <strong>Try This Next</strong>
                    <p>
                      Click "Run Planner" without changing the parameters when
                      doing it for the first time.
                    </p>
                  </div>
                  <div
                    className="controls-section"
                    style={{
                      background: 'rgba(0,0,0,0.2)',
                      padding: '1rem',
                      borderRadius: '8px',
                      marginTop: '1rem',
                    }}
                  >
                    {/* Preset Buttons */}
                    <div className="preset-buttons">
                      {Object.entries(RRT_PRESETS).map(([key, preset]) => (
                        <button
                          key={key}
                          className={`preset-btn ${
                            rrtParams.stepSize === preset.stepSize &&
                            rrtParams.maxIter === preset.maxIter &&
                            rrtParams.goalBias === preset.goalBias
                              ? 'active'
                              : ''
                          }`}
                          onClick={() =>
                            setRrtParams({
                              stepSize: preset.stepSize,
                              maxIter: preset.maxIter,
                              goalBias: preset.goalBias,
                              seed: preset.seed,
                            })
                          }
                          title={preset.description}
                        >
                          {preset.label}
                        </button>
                      ))}
                    </div>
                    <b>Parameters</b>
                    <br />- <b>Step Size:</b> How far the robot reaches in each
                    step. Larger steps jump gaps but might hit walls.
                    <br />
                    <div className="parameter-control">
                      <label>Step Size: {rrtParams.stepSize}</label>
                      <input
                        type="range"
                        min="0.01"
                        max="0.5"
                        step="0.01"
                        value={rrtParams.stepSize}
                        onChange={(e) =>
                          handleParamChange(
                            'stepSize',
                            parseFloat(e.target.value)
                          )
                        }
                      />
                    </div>
                    <br />- <b>Iterations:</b> How many attempts to make. More
                    attempts = higher chance of success but slower.
                    <br />
                    <div className="parameter-control">
                      <label>Max Iterations: {rrtParams.maxIter}</label>
                      <input
                        type="range"
                        min="1000"
                        max="20000"
                        step="1000"
                        value={rrtParams.maxIter}
                        onChange={(e) =>
                          handleParamChange('maxIter', parseInt(e.target.value))
                        }
                      />
                    </div>
                    <div className="parameter-control">
                      <label>
                        Seed: {rrtParams.seed}
                        {tutorialMode && (
                          <span
                            className="tiny-text"
                            style={{ marginLeft: '0.5rem', opacity: 0.7 }}
                          >
                            (locked in tutorial mode)
                          </span>
                        )}
                      </label>
                      <input
                        type="number"
                        min="0"
                        max="999999"
                        value={rrtParams.seed}
                        disabled={tutorialMode}
                        onChange={(e) =>
                          handleParamChange(
                            'seed',
                            parseInt(e.target.value) || 0
                          )
                        }
                        style={{
                          width: '100%',
                          padding: '0.3rem',
                          opacity: tutorialMode ? 0.5 : 1,
                        }}
                      />
                    </div>
                    <p
                      className="tiny-text"
                      style={{ marginTop: '0.3rem', marginBottom: '0.5rem' }}
                    >
                      Seed controls randomness. Same seed = same exploration
                      pattern.
                    </p>
                    <button
                      className={`primary-btn ${
                        !stats ? 'pulse-animation' : ''
                      }`}
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
                      style={{ width: '100%', marginTop: '1rem' }}
                    >
                      ▶ Run Planner
                    </button>
                  </div>
                  <br />
                  <div className="callout-box notice">
                    <strong>What to Notice</strong>
                    <p>
                      See how the tree grows in all directions? This "bushy"
                      exploration is thorough but slow. But for this "Behind the
                      Wall" target, it will likely{' '}
                      <b>
                        FAIL (the robot tip will not move to the target
                        position)
                      </b>{' '}
                      or time out. This is because this algorithm searches a
                      little bit all over the place (since it picks points to
                      extend toward randomly).
                    </p>
                  </div>
                  <br />
                  <div className="callout-box action">
                    <strong>
                      If it Fails to hit the target, try this next
                    </strong>
                    <p>
                      Increase Step Size and Max Iterations. Can you get it to
                      find the target? Notice how many nodes it needs.
                    </p>
                  </div>
                  <br />
                  You can play around with the target and rerun the planner to
                  see how the algorithm behaves in different scenarios. Note:
                  you can reset the robot and the target to starting positions
                  on the right hand side.
                  <br />
                  <br />
                  <b>Still Failing to reach the target?</b> If after tuning
                  parameters it still fails, Standard RRT's uniform random
                  exploration may not be reaching the right regions of space
                  within the time limit. The algorithm explores everywhere
                  equally rather than focusing toward the goal.
                  <br />
                  <br />
                  Try adding a <b>Joint</b> (on the right hand side) as a final
                  measure. Adding another degree of freedom gives the robot more
                  ways to bend around the obstacle.
                </p>
              </div>

              <div className="nav-buttons">
                <button onClick={() => handleStepChange(1)}>Back</button>
                <button
                  onClick={() => handleStepChange(3)}
                  disabled={!stats}
                  style={{
                    border: '1px solid rgba(255,255,255,0.2)',
                    opacity: stats ? 1 : 0.3,
                    cursor: stats ? 'pointer' : 'not-allowed',
                  }}
                >
                  Next: Optimization
                </button>
              </div>
            </div>
          )}

          {step === 3 && (
            <div className="step-content fade-in">
              <h1>3. Bi-directional RRT-Connect</h1>

              <div className="explanation">
                <h3>Meeting in the Middle</h3>
                <p>
                  Bi-directional RRT-Connect is a more efficient algorithm than
                  standard RRT.
                  <br />
                  <br />
                  The algorithm grows <b>two trees</b>: one from the start, one
                  from the goal. They aggressively try to meet in the middle.
                  <br />
                  <br />
                  Each tree takes turns in doing this: One tree takes a step
                  toward a random point and the other tree "looks" at where the
                  new extension is and tries to extend its nearest node to it.
                  Next, they swap roles and redo the same process.
                  <br />
                  <br />
                  <span>
                    This random turn-by-turn step and connect interplay might
                    seem chaotic but it is actually very efficient.
                  </span>
                  <br />
                </p>

                {tutorialMode && (
                  <div className="callout-box action">
                    <strong>Try This</strong>
                    <p>
                      Click <b>"Run Comparison"</b> to see both algorithms
                      side-by-side with identical parameters. Standard RRT runs
                      first, then RRT-Connect - so you get a true
                      apples-to-apples comparison.
                    </p>
                  </div>
                )}

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
                      to explore. Both algorithms used identical parameters for
                      a fair comparison.
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
                    {tutorialMode
                      ? 'Click "Run Comparison" to see both algorithms side-by-side.'
                      : 'To see a comparison, first run Standard RRT in step 2, then return here.'}
                  </p>
                )}
              </div>

              <div className="controls-section">
                <h3>Algorithm Parameters</h3>

                {/* Preset Buttons */}
                <div className="preset-buttons">
                  {Object.entries(RRT_PRESETS).map(([key, preset]) => (
                    <button
                      key={key}
                      className={`preset-btn ${
                        rrtParams.stepSize === preset.stepSize &&
                        rrtParams.maxIter === preset.maxIter &&
                        rrtParams.goalBias === preset.goalBias
                          ? 'active'
                          : ''
                      }`}
                      onClick={() =>
                        setRrtParams({
                          stepSize: preset.stepSize,
                          maxIter: preset.maxIter,
                          goalBias: preset.goalBias,
                          seed: preset.seed,
                        })
                      }
                      title={preset.description}
                    >
                      {preset.label}
                    </button>
                  ))}
                </div>

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
                    max="20000"
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
                <div className="parameter-control">
                  <label>
                    Seed: {rrtParams.seed}
                    {tutorialMode && (
                      <span
                        className="tiny-text"
                        style={{ marginLeft: '0.5rem', opacity: 0.7 }}
                      >
                        (locked in tutorial mode)
                      </span>
                    )}
                  </label>
                  <input
                    type="number"
                    min="0"
                    max="999999"
                    value={rrtParams.seed}
                    disabled={tutorialMode}
                    onChange={(e) =>
                      handleParamChange('seed', parseInt(e.target.value) || 0)
                    }
                    style={{
                      width: '100%',
                      padding: '0.3rem',
                      opacity: tutorialMode ? 0.5 : 1,
                    }}
                  />
                </div>
                <p className="tiny-text" style={{ marginTop: '0.3rem' }}>
                  Seed controls randomness. Same seed = same exploration
                  pattern.
                </p>
              </div>

              <div className="nav-buttons">
                <button onClick={() => handleStepChange(2)}>Back</button>
                <button
                  className="run-btn"
                  onClick={() => {
                    // In tutorial mode, run fair comparison (both algorithms)
                    if (tutorialMode) {
                      setComparisonPhase('idle') // Reset before starting
                      controllerRef.current?.runPlannerComparison()
                    } else {
                      controllerRef.current?.runPlanner()
                    }
                  }}
                  disabled={
                    comparisonPhase !== 'idle' && comparisonPhase !== 'done'
                  }
                  style={{
                    backgroundColor:
                      comparisonPhase !== 'idle' && comparisonPhase !== 'done'
                        ? '#666'
                        : '#2ecc71',
                    color: 'white',
                    fontWeight: 'bold',
                  }}
                >
                  {comparisonPhase === 'standard' &&
                    '⏳ Running Standard RRT...'}
                  {comparisonPhase === 'connect' && '⏳ Running RRT-Connect...'}
                  {(comparisonPhase === 'idle' || comparisonPhase === 'done') &&
                    (tutorialMode ? '▶ Run Comparison' : '▶ Run Planner')}
                </button>
              </div>

              <div className="callout-box notice">
                <strong>What to Notice</strong>
                <p>
                  Two trees grow toward each other - much more directed! Compare
                  the node count and time to Standard RRT.
                </p>
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

            <label className="toggle-row" style={{ marginBottom: '0.5rem' }}>
              <input
                type="checkbox"
                checked={showTree}
                onChange={(e) => {
                  setShowTree(e.target.checked)
                  controllerRef.current?.setTreeVisible(e.target.checked)
                }}
              />
              <span style={{ color: showTree ? '#2ecc71' : '#aaa' }}>
                Show Search Tree
              </span>
            </label>
            <p className="tiny-text" style={{ marginBottom: '0.8rem' }}>
              Display RRT exploration tree visualization.
            </p>

            <button
              onClick={() => {
                // Reset to tutorial default based on current step
                if (step === 1) {
                  // Greedy step: target behind wall
                  controllerRef.current?.setTargetPosition(0.39, 1.65, 2.0)
                } else if (step === 3) {
                  // RRT-Connect step: target behind gate
                  controllerRef.current?.setTargetPosition(3.31, 1.53, 1.88)
                } else {
                  // RRT steps (2) and Intro: standard RRT target position
                  controllerRef.current?.setTargetPosition(2.24, 2.59, 2.0)
                }
              }}
              style={{ width: '100%', marginBottom: '0.5rem' }}
            >
              Reset Target Position
            </button>
            <button
              onClick={() => controllerRef.current?.resetRobotPosition()}
              disabled={step === 1}
              style={{
                width: '100%',
                marginBottom: '0.5rem',
                opacity: step === 1 ? 0.4 : 1,
                cursor: step === 1 ? 'not-allowed' : 'pointer',
              }}
            >
              Reset Robot Position
            </button>
            <button
              onClick={handleResetObstacle}
              disabled={step < 2}
              style={{
                width: '100%',
                marginBottom: '0.5rem',
                opacity: step < 2 ? 0.4 : 1,
                cursor: step < 2 ? 'not-allowed' : 'pointer',
              }}
            >
              Reset Obstacle
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
                disabled={step === 1}
                style={
                  step === 1 ? { opacity: 0.4, cursor: 'not-allowed' } : {}
                }
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
                disabled={step === 1}
                style={
                  step === 1 ? { opacity: 0.4, cursor: 'not-allowed' } : {}
                }
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

          {/* Obstacle Preset System - Only editable in RRT steps */}
          <div
            className={`control-group ${
              !isObstacleEditable ? 'gated-control' : ''
            }`}
          >
            <h4>Obstacle Shape</h4>
            <div className="button-group" style={{ marginBottom: '0.5rem' }}>
              <button
                className={obstaclePreset === 'wall' ? 'active' : ''}
                onClick={() => {
                  setObstaclePreset('wall')
                  controllerRef.current?.setObstaclePreset('wall', wallDims)
                  const pos = controllerRef.current?.getObstaclePosition()
                  if (pos) setObstaclePos(pos)
                }}
                title="Simple vertical wall"
              >
                Wall
              </button>
              <button
                className={obstaclePreset === 'inverted_u' ? 'active' : ''}
                onClick={() => {
                  setObstaclePreset('inverted_u')
                  controllerRef.current?.setObstaclePreset(
                    'inverted_u',
                    gateDims
                  )
                  const pos = controllerRef.current?.getObstaclePosition()
                  if (pos) setObstaclePos(pos)
                }}
                title="Two pillars with top bar (gate)"
              >
                ∩ Gate
              </button>
              <button
                className={obstaclePreset === 'corridor' ? 'active' : ''}
                onClick={() => {
                  setObstaclePreset('corridor')
                  controllerRef.current?.setObstaclePreset(
                    'corridor',
                    corridorDims
                  )
                  const pos = controllerRef.current?.getObstaclePosition()
                  if (pos) setObstaclePos(pos)
                }}
                title="Narrow corridor"
              >
                Corridor
              </button>
            </div>

            {/* Edit Mode Toggle */}
            <div
              style={{
                display: 'flex',
                gap: '0.5rem',
                marginBottom: '0.5rem',
              }}
            >
              <button
                className={editingTarget ? 'active' : ''}
                onClick={() => {
                  setEditingTarget(true)
                  controllerRef.current?.setEditingTarget(true)
                }}
                style={{ flex: 1, fontSize: '0.8rem' }}
              >
                🎯 Move Target
              </button>
              <button
                className={!editingTarget ? 'active' : ''}
                onClick={() => {
                  setEditingTarget(false)
                  controllerRef.current?.setEditingTarget(false)
                  // Reset to translate mode when switching to obstacle editing
                  setObstacleTransformMode('translate')
                  controllerRef.current?.setObstacleTransformMode('translate')
                }}
                style={{ flex: 1, fontSize: '0.8rem' }}
              >
                📦 Edit Obstacle
              </button>
            </div>

            {/* Obstacle Transform Mode Toggle (Move/Rotate) */}
            {!editingTarget && (
              <div
                className="toggle-row"
                style={{
                  display: 'flex',
                  gap: '0.5rem',
                  marginBottom: '0.5rem',
                }}
              >
                <button
                  className={
                    obstacleTransformMode === 'translate' ? 'active' : ''
                  }
                  onClick={() => {
                    setObstacleTransformMode('translate')
                    controllerRef.current?.setObstacleTransformMode('translate')
                  }}
                  style={{ flex: 1, fontSize: '0.75rem', padding: '0.3rem' }}
                >
                  ↔ Move
                </button>
                <button
                  className={obstacleTransformMode === 'rotate' ? 'active' : ''}
                  onClick={() => {
                    setObstacleTransformMode('rotate')
                    controllerRef.current?.setObstacleTransformMode('rotate')
                  }}
                  style={{ flex: 1, fontSize: '0.75rem', padding: '0.3rem' }}
                >
                  🔄 Rotate
                </button>
              </div>
            )}

            {/* Position and Rotation Display (when editing obstacles) */}
            {!editingTarget && (
              <div
                style={{
                  fontSize: '0.75rem',
                  color: '#888',
                  marginBottom: '0.5rem',
                }}
              >
                <div>
                  Position: X={obstaclePos.x.toFixed(2)}, Z=
                  {obstaclePos.z.toFixed(2)}
                </div>
                <div>Rotation: {obstacleRotation.toFixed(0)}°</div>
              </div>
            )}

            {/* Shape-specific dimension sliders */}
            {obstaclePreset === 'wall' && (
              <div style={{ marginTop: '0.5rem' }}>
                <div className="parameter-control">
                  <label style={{ fontSize: '0.75rem' }}>
                    Width: {wallDims.width.toFixed(1)}m
                  </label>
                  <input
                    type="range"
                    min="1.0"
                    max="5.0"
                    step="0.1"
                    value={wallDims.width}
                    onChange={(e) => {
                      const width = parseFloat(e.target.value)
                      const newDims = { ...wallDims, width }
                      setWallDims(newDims)
                      controllerRef.current?.setObstaclePreset(
                        'wall',
                        newDims,
                        true
                      )
                    }}
                  />
                </div>
                <div className="parameter-control">
                  <label style={{ fontSize: '0.75rem' }}>
                    Height: {wallDims.height.toFixed(1)}m
                  </label>
                  <input
                    type="range"
                    min="1.0"
                    max="5.0"
                    step="0.1"
                    value={wallDims.height}
                    onChange={(e) => {
                      const height = parseFloat(e.target.value)
                      const newDims = { ...wallDims, height }
                      setWallDims(newDims)
                      controllerRef.current?.setObstaclePreset(
                        'wall',
                        newDims,
                        true
                      )
                    }}
                  />
                </div>
              </div>
            )}

            {obstaclePreset === 'inverted_u' && (
              <div style={{ marginTop: '0.5rem' }}>
                <div className="parameter-control">
                  <label style={{ fontSize: '0.75rem' }}>
                    Gap Width: {gateDims.gapWidth.toFixed(1)}m
                  </label>
                  <input
                    type="range"
                    min="0.3"
                    max="2.0"
                    step="0.1"
                    value={gateDims.gapWidth}
                    onChange={(e) => {
                      const gapWidth = parseFloat(e.target.value)
                      const newDims = { ...gateDims, gapWidth }
                      setGateDims(newDims)
                      controllerRef.current?.setObstaclePreset(
                        'inverted_u',
                        newDims,
                        true
                      )
                    }}
                  />
                </div>
                <div className="parameter-control">
                  <label style={{ fontSize: '0.75rem' }}>
                    Height: {gateDims.height.toFixed(1)}m
                  </label>
                  <input
                    type="range"
                    min="0.5"
                    max="4.0"
                    step="0.1"
                    value={gateDims.height}
                    onChange={(e) => {
                      const height = parseFloat(e.target.value)
                      const newDims = { ...gateDims, height }
                      setGateDims(newDims)
                      controllerRef.current?.setObstaclePreset(
                        'inverted_u',
                        newDims,
                        true
                      )
                    }}
                  />
                </div>
                <div className="parameter-control">
                  <label style={{ fontSize: '0.75rem' }}>
                    Pillar Thickness: {gateDims.pillarThickness.toFixed(1)}m
                  </label>
                  <input
                    type="range"
                    min="0.1"
                    max="1.0"
                    step="0.1"
                    value={gateDims.pillarThickness}
                    onChange={(e) => {
                      const pillarThickness = parseFloat(e.target.value)
                      const newDims = { ...gateDims, pillarThickness }
                      setGateDims(newDims)
                      controllerRef.current?.setObstaclePreset(
                        'inverted_u',
                        newDims,
                        true
                      )
                    }}
                  />
                </div>
              </div>
            )}

            {obstaclePreset === 'corridor' && (
              <div style={{ marginTop: '0.5rem' }}>
                <div className="parameter-control">
                  <label style={{ fontSize: '0.75rem' }}>
                    Corridor Width: {corridorDims.corridorWidth.toFixed(1)}m
                  </label>
                  <input
                    type="range"
                    min="0.3"
                    max="2.0"
                    step="0.1"
                    value={corridorDims.corridorWidth}
                    onChange={(e) => {
                      const corridorWidth = parseFloat(e.target.value)
                      const newDims = { ...corridorDims, corridorWidth }
                      setCorridorDims(newDims)
                      controllerRef.current?.setObstaclePreset(
                        'corridor',
                        newDims,
                        true
                      )
                    }}
                  />
                </div>
                <div className="parameter-control">
                  <label style={{ fontSize: '0.75rem' }}>
                    Length: {corridorDims.length.toFixed(1)}m
                  </label>
                  <input
                    type="range"
                    min="0.5"
                    max="3.0"
                    step="0.1"
                    value={corridorDims.length}
                    onChange={(e) => {
                      const length = parseFloat(e.target.value)
                      const newDims = { ...corridorDims, length }
                      setCorridorDims(newDims)
                      controllerRef.current?.setObstaclePreset(
                        'corridor',
                        newDims,
                        true
                      )
                    }}
                  />
                </div>
                <div className="parameter-control">
                  <label style={{ fontSize: '0.75rem' }}>
                    Height: {corridorDims.height.toFixed(1)}m
                  </label>
                  <input
                    type="range"
                    min="0.5"
                    max="3.0"
                    step="0.1"
                    value={corridorDims.height}
                    onChange={(e) => {
                      const height = parseFloat(e.target.value)
                      const newDims = { ...corridorDims, height }
                      setCorridorDims(newDims)
                      controllerRef.current?.setObstaclePreset(
                        'corridor',
                        newDims,
                        true
                      )
                    }}
                  />
                </div>
              </div>
            )}
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
                    // Reset arm length, joint width, and tip size to current values when resetting joints
                    setArmLength(controllerRef.current.getArmLength())
                    setJointWidth(controllerRef.current.getJointWidth())
                    setTipSize(controllerRef.current.getTipSize())
                  }
                }}
              >
                Reset
              </button>
            </div>

            {/* Arm Length Slider */}
            <div className="parameter-control" style={{ marginTop: '0.75rem' }}>
              <label style={{ fontSize: '0.75rem' }}>
                Arm Length: {armLength.toFixed(1)}x
              </label>
              <input
                type="range"
                min="0.5"
                max="2.0"
                step="0.1"
                value={armLength}
                onChange={(e) => {
                  const scale = parseFloat(e.target.value)
                  setArmLength(scale)
                  controllerRef.current?.setArmLength(scale)
                }}
              />
              <p
                className="tiny-text"
                style={{ marginTop: '0.2rem', color: '#888' }}
              >
                Shorter arms can fit through narrower gaps.
              </p>
            </div>

            {/* Joint Width Slider */}
            <div className="parameter-control" style={{ marginTop: '0.75rem' }}>
              <label style={{ fontSize: '0.75rem' }}>
                Joint Size: {jointWidth.toFixed(1)}x
              </label>
              <input
                type="range"
                min="0.2"
                max="1.5"
                step="0.1"
                value={jointWidth}
                onChange={(e) => {
                  const scale = parseFloat(e.target.value)
                  setJointWidth(scale)
                  controllerRef.current?.setJointWidth(scale)
                }}
              />
              <p
                className="tiny-text"
                style={{ marginTop: '0.2rem', color: '#888' }}
              >
                Thinner joints/arms reduce collision volume.
              </p>
            </div>

            {/* Tip Size Slider */}
            <div className="parameter-control" style={{ marginTop: '0.75rem' }}>
              <label style={{ fontSize: '0.75rem' }}>
                Tip Size: {tipSize.toFixed(1)}x
              </label>
              <input
                type="range"
                min="0.1"
                max="2.0"
                step="0.1"
                value={tipSize}
                onChange={(e) => {
                  const scale = parseFloat(e.target.value)
                  setTipSize(scale)
                  controllerRef.current?.setTipSize(scale)
                }}
              />
              <p
                className="tiny-text"
                style={{ marginTop: '0.2rem', color: '#888' }}
              >
                Smaller tip can fit through tighter spaces.
              </p>
            </div>
          </div>

          {/* Camera Settings */}
          <div
            className={`control-group ${
              !isInteractiveStep ? 'gated-control' : ''
            }`}
          >
            <h4>Camera Settings</h4>

            <div
              className="parameter-control"
              style={{ marginBottom: '0.5rem' }}
            >
              <label style={{ fontSize: '0.75rem' }}>
                Zoom: {cameraZoom.toFixed(1)}
              </label>
              <input
                type="range"
                min="2"
                max="20"
                step="0.5"
                value={cameraZoom}
                onChange={(e) => {
                  const zoom = parseFloat(e.target.value)
                  setCameraZoom(zoom)
                  controllerRef.current?.setCameraZoom(zoom)
                }}
              />
            </div>

            <div
              className="parameter-control"
              style={{ marginBottom: '0.5rem' }}
            >
              <label style={{ fontSize: '0.75rem' }}>
                Orbit (Azimuth): {cameraAzimuth.toFixed(0)}°
              </label>
              <input
                type="range"
                min="0"
                max="360"
                step="1"
                value={cameraAzimuth}
                onChange={(e) => {
                  const azimuth = parseFloat(e.target.value)
                  setCameraAzimuth(azimuth)
                  controllerRef.current?.setCameraAngle(azimuth, cameraPolar)
                }}
              />
            </div>

            <div className="parameter-control">
              <label style={{ fontSize: '0.75rem' }}>
                Elevation (Polar): {cameraPolar.toFixed(0)}°
              </label>
              <input
                type="range"
                min="0"
                max="90"
                step="1"
                value={cameraPolar}
                onChange={(e) => {
                  const polar = parseFloat(e.target.value)
                  setCameraPolar(polar)
                  controllerRef.current?.setCameraAngle(cameraAzimuth, polar)
                }}
              />
            </div>
          </div>
        </div>
      </div>

      {/* Bottom HUD Container - Stats, Legend, and Controls */}
      <div className="bottom-hud">
        {/* Top row: Status Bar + Tree Legend side by side */}
        <div className="bottom-hud-row">
          {/* Status Bar - Shows stats and failure feedback */}
          {((step === 1 && stats && !stats.success) || step >= 2) && stats && (
            <div className="status-bar fade-in">
              {/* Stats Section - Only show for RRT steps (2+), not Greedy */}
              {step >= 2 && (
                <div className="status-bar-stats">
                  <div className="status-item">
                    <span className="status-label">Status</span>
                    <span
                      className={`status-value ${
                        stats.success ? 'success' : 'error'
                      }`}
                    >
                      {stats.success
                        ? 'CONVERGED'
                        : stats.time > 0
                        ? 'FAILED'
                        : 'PLANNING'}
                    </span>
                  </div>
                  <div className="status-item">
                    <span className="status-label">Time</span>
                    <span className="status-value">{stats.time} ms</span>
                  </div>
                  <div className="status-item">
                    <span className="status-label">Nodes</span>
                    <span className="status-value">{stats.nodes}</span>
                  </div>
                </div>
              )}

              {/* Failure Feedback Section */}
              {!stats.success && stats.failureReason && (
                <div className="status-bar-feedback">
                  <div className="feedback-reason">
                    <span className="feedback-icon">!</span>
                    <span className="feedback-message">
                      {getFailureFeedback(
                        stats.failureReason,
                        stats.isGreedy,
                        step
                      )?.message || 'Planning failed'}
                    </span>
                  </div>
                  <div className="feedback-suggestions">
                    <span className="suggestions-label">Try:</span>
                    {getFailureFeedback(
                      stats.failureReason,
                      stats.isGreedy,
                      step
                    )?.suggestions.map((s, i) => (
                      <span key={i} className="suggestion-chip">
                        {s}
                      </span>
                    ))}
                  </div>
                </div>
              )}
            </div>
          )}

          {/* Tree Legend - Only visible in RRT-Connect step */}
          {step === 3 && (
            <div className="tree-legend fade-in">
              <span className="legend-title">RRT-Connect</span>
              <div className="legend-items">
                <div className="legend-item">
                  <span
                    className="legend-line"
                    style={{ background: '#55ff55' }}
                  />
                  <span style={{ color: '#55ff55' }}>Start Tree</span>
                </div>
                <div className="legend-item">
                  <span
                    className="legend-line"
                    style={{ background: '#55aaff' }}
                  />
                  <span style={{ color: '#55aaff' }}>Goal Tree</span>
                </div>
                {showComparison && (
                  <>
                    <span className="legend-separator">│</span>
                    <div className="legend-item">
                      <span
                        className="legend-line"
                        style={{ background: '#ff5555' }}
                      />
                      <span style={{ color: '#ff5555' }}>Std RRT</span>
                    </div>
                  </>
                )}
              </div>
            </div>
          )}
        </div>

        {/* Controls Help */}
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
    </div>
  )
}

export default App
