import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { TransformControls } from 'three/addons/controls/TransformControls.js'
import { Robot } from './Robot.ts'
import { RRTPlanner, type RRTParams } from './RRTPlanner.ts'
import type { WorkerInput, WorkerOutput } from './rrtWorker.ts'

export type PlannerStats = {
  time: number
  nodes: number
  success: boolean
}

// Serialized tree node for visualization (from worker)
interface SerializedNode {
  angles: number[]
  parentIndex: number | null
}

export class SceneController {
  public scene: THREE.Scene
  public camera: THREE.PerspectiveCamera
  public renderer: THREE.WebGLRenderer
  public robot: Robot
  public ghostRobot: Robot
  public planner: RRTPlanner
  public controls: OrbitControls

  // Objects
  public targetMesh: THREE.Mesh
  public obstacle: THREE.Mesh
  private treeMesh: THREE.LineSegments | null = null
  private ghostTreeMesh: THREE.LineSegments | null = null
  private transformControl: TransformControls

  // State
  private plannedPath: number[][] = []
  private pathIndex = 0
  private isPlaying = false
  private isAnimatingTree = false
  private collisionDebugGroup: THREE.Group

  // Web Worker for RRT planning (keeps UI responsive)
  private worker: Worker | null = null
  private isWorkerPlanning = false

  private algorithm: 'greedy' | 'rrt' | 'rrt-standard' = 'rrt'
  private rrtParams: RRTParams = {
    stepSize: 0.2,
    maxIter: 20000,
    goalBias: 0.1,
    algorithm: 'connect',
  }

  // Callbacks to update React UI
  public onStatsUpdate: ((stats: PlannerStats) => void) | null = null
  public onTargetMove:
    | ((pos: { x: number; y: number; z: number }) => void)
    | null = null

  constructor(canvas: HTMLCanvasElement) {
    // 1. Setup ThreeJS
    this.scene = new THREE.Scene()
    this.scene.background = new THREE.Color(0x202020)

    this.camera = new THREE.PerspectiveCamera(
      75,
      window.innerWidth / window.innerHeight,
      0.1,
      1000
    )
    this.camera.position.set(3, 3, 6) // Zoomed out (was 5)
    this.camera.lookAt(0, 1, 0)

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true })
    this.renderer.setSize(window.innerWidth, window.innerHeight)

    // Lighting
    const ambient = new THREE.AmbientLight(0xffffff, 0.6)
    const dir = new THREE.DirectionalLight(0xffffff, 1)
    dir.position.set(5, 10, 7)
    this.scene.add(ambient, dir, new THREE.GridHelper(10, 10))

    // 2. Setup Robot & Environment
    this.robot = new Robot()
    this.scene.add(this.robot.sceneObject)
    // Ensure initial color is correct (not red)
    this.robot.setOverrideColor(null)

    // Ghost
    this.ghostRobot = new Robot()
    this.makeGhost(this.ghostRobot.sceneObject)
    this.scene.add(this.ghostRobot.sceneObject)
    this.ghostRobot.sceneObject.visible = false // Hidden by default in new plan

    // Target
    this.targetMesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.15),
      new THREE.MeshBasicMaterial({
        color: 0xff0000,
        transparent: true,
        opacity: 0.8,
      })
    )
    this.targetMesh.position.set(2, 2, 0)
    this.scene.add(this.targetMesh)

    // Add a pulsate animation to the target mesh
    this.targetMesh.userData.pulsate = true
    this.targetMesh.userData.time = 0

    // Obstacle
    this.obstacle = new THREE.Mesh(
      new THREE.BoxGeometry(1, 2, 0.5),
      new THREE.MeshStandardMaterial({ color: 0x888888 })
    )
    this.obstacle.position.set(0.5, 1, 1)
    this.scene.add(this.obstacle)
    this.scene.updateMatrixWorld() // Important for math

    this.planner = new RRTPlanner(this.robot, this.obstacle)

    // Initialize Web Worker for RRT planning
    this.worker = new Worker(new URL('./rrtWorker.ts', import.meta.url), {
      type: 'module',
    })
    this.worker.onmessage = (e: MessageEvent<WorkerOutput>) => {
      this.handleWorkerResult(e.data)
    }
    // Handle worker errors (crashes, unhandled exceptions, etc.)
    // This prevents the UI from getting stuck in "planning" state forever
    this.worker.onerror = (e: ErrorEvent) => {
      console.error('[SceneController] Worker error:', e.message)
      this.isWorkerPlanning = false
      // Notify UI of failure
      if (this.onStatsUpdate) {
        this.onStatsUpdate({
          time: 0,
          nodes: 0,
          success: false,
        })
      }
      // Visual feedback
      this.robot.setOverrideColor(0xff0000)
      setTimeout(() => {
        this.robot.setOverrideColor(null)
      }, 500)
    }

    // 3. Controls
    this.controls = new OrbitControls(this.camera, canvas)
    this.transformControl = new TransformControls(this.camera, canvas)
    this.transformControl.addEventListener('dragging-changed', (event) => {
      this.controls.enabled = !event.value
      // On Drag End -> Plan
      if (!event.value && this.algorithm === 'greedy') this.runPlanner()
    })

    // Live Ghost Update while dragging - ONLY in RRT mode if desired, or Greedy mode live update
    this.transformControl.addEventListener('change', () => {
      if (this.onTargetMove) {
        this.onTargetMove({
          x: this.targetMesh.position.x,
          y: this.targetMesh.position.y,
          z: this.targetMesh.position.z,
        })
      }

      if (this.algorithm === 'greedy') {
        // Real-time update for Greedy
        this.runPlanner()
      } else {
        // For RRT mode, we don't want to re-plan every frame while dragging (too heavy).
        // But we MIGHT want to visualize the target validity?
        // For now, do nothing until drag ends.
      }
    })

    this.transformControl.attach(this.targetMesh)
    this.scene.add(this.transformControl.getHelper())

    // Setup Collision Debug Group
    this.collisionDebugGroup = new THREE.Group()
    this.scene.add(this.collisionDebugGroup)
    this.collisionDebugGroup.visible = false // Hidden by default

    // Start Loop
    this.animate()
  }

  public animateCameraIntro() {
    // Reverse direction: start from left (-), end at right (+)
    // But user asked to start "from other side of wall" and end "not full 180, but 120"
    // Let's interpret: Start at +60 deg, end at -60 deg? Or Start at -120, end at -60?
    // "Start from other side of wall": The wall is at X=0.5, Z=1. The robot is at 0,0,0.
    // Default camera was (3,3,6).
    // Let's try a sweep from +60 degrees to -60 degrees (Total 120 deg arc)

    const radius = 6
    const height = 3
    // 60 degrees in radians = PI/3
    const startAngle = Math.PI / 3
    const endAngle = -Math.PI / 3

    const duration = 3000 // 3 seconds
    const start = performance.now()

    const pan = () => {
      const now = performance.now()
      const t = Math.min((now - start) / duration, 1)
      // Ease out cubic
      const ease = 1 - Math.pow(1 - t, 3)

      const angle = startAngle + (endAngle - startAngle) * ease
      const x = Math.sin(angle) * radius
      const z = Math.cos(angle) * radius

      this.camera.position.set(x, height, z)
      this.camera.lookAt(0, 1, 0)

      if (t < 1) {
        requestAnimationFrame(pan)
      }
    }
    pan()
  }

  public animateCameraOutro() {
    const radius = 8 // Zoomed out further
    const height = 4 // Higher up
    // Full 360 orbit
    const startAngle = 0
    const endAngle = Math.PI * 2

    const duration = 8000 // 8 seconds
    const start = performance.now()

    const orbit = () => {
      const now = performance.now()
      const t = Math.min((now - start) / duration, 1)
      // Ease in-out cubic
      // const ease = t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2
      // Linear is better for continuous rotation
      const ease = t

      const angle = startAngle + (endAngle - startAngle) * ease
      const x = Math.sin(angle) * radius
      const z = Math.cos(angle) * radius

      this.camera.position.set(x, height, z)
      this.camera.lookAt(0, 1, 0)

      if (t < 1) {
        requestAnimationFrame(orbit)
      }
    }
    orbit()
  }

  // --- NEW: DYNAMIC JOINTS ---
  public addJoint() {
    this.robot.addJoint()
    this.ghostRobot.addJoint()

    // Re-apply ghost material
    this.makeGhost(this.ghostRobot.sceneObject)

    // Re-run planner if auto-update is desired, or just reset
    if (this.algorithm === 'greedy') {
      this.runPlanner()
    }
  }

  public removeJoint() {
    this.robot.removeJoint()
    this.ghostRobot.removeJoint()

    // Re-apply ghost material
    this.makeGhost(this.ghostRobot.sceneObject)

    // Force scene graph update immediately
    this.scene.updateMatrixWorld(true)

    // Re-run planner if auto-update is desired, or just reset
    if (this.algorithm === 'greedy') {
      this.runPlanner()
    }
  }

  public resetJoints() {
    this.robot.resetConfig()
    this.ghostRobot.resetConfig()
    this.makeGhost(this.ghostRobot.sceneObject)
    if (this.algorithm === 'greedy') {
      this.runPlanner()
    }
  }

  /**
   * Returns the number of moving joints (degrees of freedom) on the robot.
   * Useful for displaying in the UI.
   */
  public getJointCount(): number {
    return this.robot.getDoF()
  }

  // Reset robot angles to home position (all zeros)
  public resetRobotPosition() {
    const zeros = new Array(this.robot.getDoF()).fill(0)
    this.robot.setAngles(zeros)
    this.ghostRobot.setAngles(zeros)
    if (this.algorithm === 'greedy') {
      this.runPlanner()
    }
  }

  // --- NEW: INTERACTION CONTROL ---
  public setInteraction(enabled: boolean) {
    this.controls.enabled = enabled
    this.transformControl.enabled = enabled

    if (enabled) {
      this.transformControl.attach(this.targetMesh)
    } else {
      this.transformControl.detach()
    }
  }

  public setTargetPosition(x: number, y: number, z: number) {
    this.targetMesh.position.set(x, y, z)
    if (this.onTargetMove) {
      this.onTargetMove({ x, y, z })
    }
    if (this.algorithm === 'greedy') {
      this.runPlanner()
    }
  }

  // --- NEW: SCENARIO MANAGER ---
  public setScenario(type: 'easy' | 'medium' | 'hard') {
    // Reset everything
    this.robot.setAngles([0, 0, 0, 0, 0])
    this.plannedPath = []
    if (this.treeMesh) {
      this.scene.remove(this.treeMesh)
      this.treeMesh.geometry.dispose()
      this.treeMesh = null
    }

    if (type === 'easy') {
      // No obstacles, easy target
      this.obstacle.position.set(0, -10, 0) // Hide wall underground
      this.targetMesh.position.set(2, 2, 0)
    } else if (type === 'medium') {
      // A standard wall that blocks direct line of sight
      // Was: y=1.5, scale.y=2.0. Let's make it a bit shorter to easily see "going over"
      this.obstacle.position.set(1.0, 1.0, 0.5) // Lower center
      this.obstacle.scale.set(0.2, 1.5, 2.0) // Shorter height
      this.targetMesh.position.set(2.0, 1.5, 0.5) // Just behind it
    } else if (type === 'hard') {
      // The "Mailbox" Slot
      // A wall with the target deep inside/behind, forcing the arm to snake around
      this.obstacle.position.set(0.8, 1.5, 0)
      this.obstacle.scale.set(0.5, 2.5, 2.5) // Huge wall
      this.targetMesh.position.set(0, 1.5, 1.5) // 90 degrees to the side, behind wall
    }

    this.obstacle.updateMatrixWorld() // Critical update
    if (this.algorithm === 'greedy') {
      this.runPlanner() // Auto-run for effect
    }
  }

  // --- NEW: Dynamic Wall Adjustment ---
  public setObstacleHeight(height: number) {
    // height is a scale factor essentially, or raw height?
    // Let's treat it as raw height. Original box is 2 units high.
    // If we scale Y, we must also move Y position so it stays on the floor (y=0)
    // Original center is y=1, height=2.

    // Scale.y = newHeight / 2 (since geometry height is 2)
    // Actually base geometry is 2.
    const scale = height / 2.0
    this.obstacle.scale.y = scale
    this.obstacle.position.y = height / 2.0
    this.obstacle.updateMatrixWorld()

    // Re-run current planner logic
    if (this.algorithm === 'greedy') {
      this.runPlanner()
    }
  }

  public setAlgorithm(
    mode: 'greedy' | 'rrt' | 'rrt-standard',
    reset: boolean = false
  ) {
    console.log(
      '[DEBUG] setAlgorithm called. Mode:',
      mode,
      'Reset:',
      reset,
      'Current:',
      this.algorithm
    )

    // NEW: Snapshot if transitioning FROM Standard RRT TO Connect
    if (this.algorithm === 'rrt-standard' && mode === 'rrt') {
      this.snapshotStandardTree()
    } else if (mode !== 'rrt') {
      // If going BACK to standard or Greedy, clear the ghost
      this.clearGhostTree()
    }

    this.algorithm = mode
    console.log('[DEBUG] Algorithm now set to:', this.algorithm)
    // Hide ghost in both modes as per new plan (Wait, plan said hide ghost robot, not ghost tree)
    // Ghost Robot (the green one) is different from Ghost Tree.
    this.ghostRobot.sceneObject.visible = false

    // RESET COLOR: Clear any leftover collision color from Greedy step
    this.robot.setOverrideColor(null)

    // Reset worker planning flag when switching algorithms to prevent stuck state
    this.isWorkerPlanning = false

    // Clear trees when switching
    if (this.treeMesh) {
      this.scene.remove(this.treeMesh)
      this.treeMesh.geometry.dispose()
      this.treeMesh = null
    }

    if (reset) {
      this.robot.setAngles([0, 0, 0, 0, 0]) // Reset to home
      this.robot.setOverrideColor(null) // Reset color
      // Note: We do NOT reset the target position, so it stays in the "stuck" spot
    }

    // APPLY PRESETS
    if (mode === 'rrt-standard' || mode === 'rrt') {
      // Weak defaults to force failure/struggle initially
      const weakParams: RRTParams = {
        stepSize: 0.05, // Tiny steps = slow exploration
        maxIter: 5000, // Low iterations = likely timeout
        goalBias: 0.05, // Low bias = less guidance
        algorithm: mode === 'rrt-standard' ? 'standard' : 'connect',
      }
      this.rrtParams = weakParams
      // Update UI state if possible? (This is one-way, React won't see this unless we sync back.
      // For now, we just set internal state. Ideally App.tsx calls updateRRTParams on load)
      // But wait, App.tsx controls the state. We should probably let App.tsx handle the presets.
      // IGNORE THIS BLOCK - Logic moved to App.tsx for state sync.
    }

    // GREEDY runs immediately. RRT waits for button press.
    if (mode === 'greedy') {
      // IMPORTANT: Force a re-render of the scene state before planning to ensure matrices are updated
      this.scene.updateMatrixWorld(true)
      this.runPlanner()
    } else {
      // RRT modes: Just reset stats, don't run yet
      if (this.onStatsUpdate) {
        this.onStatsUpdate({ time: 0, nodes: 0, success: false })
      }
    }
  }

  public updateRRTParams(params: RRTParams) {
    this.rrtParams = params
    // No auto-run. User must click "Run Planner".
  }

  // --- WORKER HELPERS ---

  // Serialize obstacle bounding box for worker
  private getObstacleBox(): {
    min: [number, number, number]
    max: [number, number, number]
  } {
    const box = new THREE.Box3().setFromObject(this.obstacle)
    return {
      min: [box.min.x, box.min.y, box.min.z],
      max: [box.max.x, box.max.y, box.max.z],
    }
  }

  // Handle results from the RRT worker
  private handleWorkerResult(result: WorkerOutput) {
    this.isWorkerPlanning = false

    // Log any errors from the worker
    if (result.error) {
      console.error(`[SceneController] Worker error: ${result.error}`)
    }

    console.log(
      `[SceneController] Worker finished in ${result.time}ms. Success: ${result.success}`
    )

    if (!result.success) {
      // VISUAL FEEDBACK FOR FAILURE
      this.robot.setOverrideColor(0xff0000)
      setTimeout(() => {
        this.robot.setOverrideColor(null)
      }, 500)
    }

    // 1. Visualize Tree from serialized data
    this.visualizeSearchTreeFromData(result.treeData)

    // 2. Report Stats to React
    if (this.onStatsUpdate) {
      this.onStatsUpdate({
        time: result.time,
        nodes: result.treeData.length,
        success: result.success,
      })
    }

    // 3. Execute Path (DELAY UNTIL ANIMATION DONE)
    if (result.path) {
      this.plannedPath = result.path
      this.pathIndex = 0
    }
  }

  // Visualize tree from serialized worker data
  private visualizeSearchTreeFromData(treeData: SerializedNode[]) {
    // 1. CLEANUP: Remove old mesh
    if (this.treeMesh) {
      this.scene.remove(this.treeMesh)
      this.treeMesh.geometry.dispose()
      if (Array.isArray(this.treeMesh.material)) {
        this.treeMesh.material.forEach((m) => m.dispose())
      } else {
        this.treeMesh.material.dispose()
      }
      this.treeMesh.children.forEach((c) => {
        if (c instanceof THREE.Points) {
          c.geometry.dispose()
          if (Array.isArray(c.material)) {
            c.material.forEach((m) => m.dispose())
          } else {
            c.material.dispose()
          }
        }
      })
      this.treeMesh = null
    }

    if (!treeData || treeData.length === 0) return

    // 2. GENERATE LINES (The Branches)
    const points: THREE.Vector3[] = []
    const limit = Math.min(treeData.length, 20000)

    for (let i = 0; i < limit; i++) {
      const node = treeData[i]
      if (!node || node.parentIndex === null) continue

      const parentNode = treeData[node.parentIndex]
      if (!parentNode) continue

      // Calculate 3D position of the Tip for start and end of the branch
      const startPos = this.robot.getTipPosition(parentNode.angles)
      const endPos = this.robot.getTipPosition(node.angles)

      if (isNaN(startPos.x) || isNaN(endPos.x)) continue

      points.push(startPos)
      points.push(endPos)
    }

    console.log(
      `[SceneController] Visualizing Tree from worker: ${treeData.length} nodes, ${points.length} vertices`
    )

    const lineGeo = new THREE.BufferGeometry().setFromPoints(points)
    const lineMat = new THREE.LineBasicMaterial({
      color: 0x55ff55,
      transparent: true,
      opacity: 0.6,
      depthWrite: false,
    })

    this.treeMesh = new THREE.LineSegments(lineGeo, lineMat)
    this.treeMesh.geometry.setDrawRange(0, 0) // Start hidden for animation

    // 3. GENERATE POINTS (The Nodes)
    if (treeData.length < 5000) {
      const dotPoints: THREE.Vector3[] = []
      for (let i = 0; i < limit; i++) {
        const node = treeData[i]
        if (node) {
          dotPoints.push(this.robot.getTipPosition(node.angles))
        }
      }

      const dotGeo = new THREE.BufferGeometry().setFromPoints(dotPoints)
      const dotMat = new THREE.PointsMaterial({
        color: 0xccffcc,
        size: 0.04,
        transparent: true,
        opacity: 0.5,
        sizeAttenuation: true,
      })

      const dots = new THREE.Points(dotGeo, dotMat)
      dots.geometry.setDrawRange(0, 0)
      this.treeMesh.add(dots)
    }

    this.scene.add(this.treeMesh)
    this.isAnimatingTree = true
  }

  // --- NEW: DEBUG TOGGLE ---
  // Visualize the raw math used in checkCollision
  public toggleCollisionDebug(visible: boolean) {
    this.collisionDebugGroup.visible = visible
    if (visible) {
      this.updateCollisionDebugLines()
    }
  }

  // Draws the "Skeleton" lines that we actually check against the box
  // UPDATE: Visualizing Volumes instead of Lines
  private updateCollisionDebugLines() {
    this.collisionDebugGroup.clear()

    const currentAngles = this.robot.getCurrentAngles()
    // Access public getJointPositions (Ensure you updated Robot.ts!)
    const joints = this.robot.getJointPositions(
      currentAngles
    ) as THREE.Vector3[]

    // The robot's physical thickness (matches Robot.ts)
    // These values should match the actual collision radii used in Robot.ts:
    //   - Self-collision: ARM_WIDTH/2 + SELF_COLLISION_MARGIN = 0.15 + 0.02 = 0.17
    //   - Obstacle collision: ARM_WIDTH/2 + OBSTACLE_COLLISION_MARGIN = 0.15 + 0.15 = 0.30
    // We visualize the OBSTACLE collision radius (larger) for clarity
    const ARM_WIDTH = this.robot.ARM_WIDTH // 0.3
    const OBSTACLE_MARGIN = 0.15
    const COLLISION_RADIUS = ARM_WIDTH / 2 + OBSTACLE_MARGIN // 0.30
    const JOINT_RADIUS = this.robot.JOINT_RADIUS + OBSTACLE_MARGIN // 0.4 + 0.15 = 0.55

    // 1. DRAW CYLINDERS (Segments)
    // Matches the loop in Robot.ts: 0 to length-1
    for (let i = 0; i < joints.length - 1; i++) {
      // Only draw if there is a "visualLength" equivalent check
      // We assume basic robot structure has length for all main links
      if (!joints[i] || !joints[i + 1]) continue

      const start = joints[i]
      const end = joints[i + 1]

      const distance = start.distanceTo(end)
      if (distance < 0.01) continue // Skip zero-length offsets

      const geometry = new THREE.CylinderGeometry(
        COLLISION_RADIUS,
        COLLISION_RADIUS,
        distance,
        8
      )
      geometry.rotateX(Math.PI / 2)

      const material = new THREE.MeshBasicMaterial({
        color: 0xff00ff,
        wireframe: true,
        transparent: true,
        opacity: 0.3,
      })

      const mesh = new THREE.Mesh(geometry, material)
      const midPoint = new THREE.Vector3()
        .addVectors(start, end)
        .multiplyScalar(0.5)
      mesh.position.copy(midPoint)
      mesh.lookAt(end)

      this.collisionDebugGroup.add(mesh)
    }

    // 2. DRAW SPHERES (Joints)
    // Matches step 4 in Robot.ts
    joints.forEach((pos, i) => {
      // Heuristic: if it's not the first one (Base is cylinder usually)
      // Robot.ts checks revolute + tip. Base is fixed but usually covered by cylinder.
      // Let's draw for all index > 0 just to be safe and see everything
      if (i > 0) {
        const geo = new THREE.SphereGeometry(JOINT_RADIUS, 8, 8)
        const mat = new THREE.MeshBasicMaterial({
          color: 0x00ffff, // Cyan for joints
          wireframe: true,
          transparent: true,
          opacity: 0.3,
        })
        const mesh = new THREE.Mesh(geo, mat)
        mesh.position.copy(pos)
        this.collisionDebugGroup.add(mesh)
      }
    })

    // Draw the Obstacle Box (Yellow Wireframe)
    const box = new THREE.Box3().setFromObject(this.obstacle)
    box.expandByScalar(0.2) // Visualizing the padding
    const helper = new THREE.Box3Helper(box, 0xffff00)
    this.collisionDebugGroup.add(helper)
  }

  public runPlanner() {
    console.log('[DEBUG] runPlanner called. Algorithm:', this.algorithm)
    console.log('Planning...')
    const startAngles = this.robot.getCurrentAngles()

    // Clear old visual
    if (this.treeMesh) {
      this.scene.remove(this.treeMesh)
      this.treeMesh.geometry.dispose()
      this.treeMesh = null
    }

    if (this.algorithm === 'greedy') {
      // The greedy approach (runs on main thread - fast enough)
      const start = performance.now()
      const sol = this.robot.calculateIK(this.targetMesh.position)
      this.robot.setAngles(sol)

      // Check collision
      const hit = this.robot.checkCollision(sol, this.obstacle)

      // Visual Feedback for Hit
      if (hit) {
        this.robot.setOverrideColor(0xff0000) // Red
      } else {
        this.robot.setOverrideColor(null) // Restore
      }

      // Report
      if (this.onStatsUpdate) {
        this.onStatsUpdate({
          time: Math.round(performance.now() - start),
          nodes: 0,
          success: !hit,
        })
      }

      this.isPlaying = false
      this.plannedPath = []
      return
    }

    // --- RRT MODES: Use Web Worker to keep UI responsive ---

    // Prevent multiple simultaneous planning requests
    if (this.isWorkerPlanning) {
      console.log('[SceneController] Worker already planning, ignoring request')
      return
    }

    // Ensure robot color is reset
    this.robot.setOverrideColor(null)

    // Update params with current mode
    const effectiveParams: RRTParams = {
      ...this.rrtParams,
      algorithm: this.algorithm === 'rrt-standard' ? 'standard' : 'connect',
    }

    // LOGGING
    console.log('[SceneController] Starting Plan via Worker:', {
      algorithm: this.algorithm,
      startAngles,
      targetPos: this.targetMesh.position,
      params: effectiveParams,
    })

    // Send planning request to worker
    if (this.worker) {
      this.isWorkerPlanning = true

      const workerInput: WorkerInput = {
        startAngles,
        targetPos: {
          x: this.targetMesh.position.x,
          y: this.targetMesh.position.y,
          z: this.targetMesh.position.z,
        },
        params: effectiveParams,
        robotConfig: this.robot.CONFIG,
        obstacleBox: this.getObstacleBox(),
      }

      this.worker.postMessage(workerInput)

      // Show "planning" state in UI
      if (this.onStatsUpdate) {
        this.onStatsUpdate({
          time: 0,
          nodes: 0,
          success: false,
        })
      }
    } else {
      // Fallback to main thread if worker not available
      console.warn('[SceneController] Worker not available, using main thread')
      this.runPlannerMainThread(startAngles, effectiveParams)
    }
  }

  // Fallback: Run planner on main thread (blocks UI)
  private runPlannerMainThread(
    startAngles: number[],
    effectiveParams: RRTParams
  ) {
    const start = performance.now()

    const path = this.planner.plan(
      startAngles,
      this.targetMesh.position,
      effectiveParams
    )
    const end = performance.now()
    console.log(
      `[SceneController] Planning finished in ${
        end - start
      }ms. Success: ${!!path}`
    )

    if (!path) {
      this.robot.setOverrideColor(0xff0000)
      setTimeout(() => {
        this.robot.setOverrideColor(null)
      }, 500)
    }

    // Visualize Tree (using old method for main thread planner)
    this.visualizeSearchTree()

    if (this.onStatsUpdate) {
      this.onStatsUpdate({
        time: Math.round(end - start),
        nodes: this.planner.lastTrees.length,
        success: !!path,
      })
    }

    if (path) {
      this.plannedPath = path
      this.pathIndex = 0
    }
  }

  private visualizeSearchTree() {
    // 1. CLEANUP: Remove old mesh to prevent memory leaks and visual artifacts
    if (this.treeMesh) {
      this.scene.remove(this.treeMesh)
      this.treeMesh.geometry.dispose()
      if (Array.isArray(this.treeMesh.material)) {
        this.treeMesh.material.forEach((m) => m.dispose())
      } else {
        this.treeMesh.material.dispose()
      }

      // Cleanup children (the dots)
      this.treeMesh.children.forEach((c) => {
        if (c instanceof THREE.Points) {
          c.geometry.dispose()
          if (Array.isArray(c.material)) {
            c.material.forEach((m) => m.dispose())
          } else {
            c.material.dispose()
          }
        }
      })
      this.treeMesh = null
    }

    const tree = this.planner.lastTrees
    if (!tree || tree.length === 0) return

    // 2. GENERATE LINES (The Branches)
    const points: THREE.Vector3[] = []
    const limit = Math.min(tree.length, 20000) // Safety cap

    for (let i = 1; i < limit; i++) {
      const node = tree[i]
      // Skip roots or malformed nodes
      if (!node || !node.parent) continue

      // Calculate 3D position of the Tip for start and end of the branch
      const startPos = this.robot.getTipPosition(node.parent.angles)
      const endPos = this.robot.getTipPosition(node.angles)

      // Check for NaNs just in case
      if (isNaN(startPos.x) || isNaN(endPos.x)) continue

      points.push(startPos)
      points.push(endPos)
    }

    console.log(
      `[SceneController] Visualizing Tree: ${tree.length} nodes, ${points.length} vertices`
    )

    const lineGeo = new THREE.BufferGeometry().setFromPoints(points)
    const lineMat = new THREE.LineBasicMaterial({
      color: 0x55ff55, // Brighter Green
      transparent: true,
      opacity: 0.6, // Much more visible (was 0.15)
      depthWrite: false,
    })

    this.treeMesh = new THREE.LineSegments(lineGeo, lineMat)
    this.treeMesh.geometry.setDrawRange(0, 0) // Start hidden for animation

    // 3. GENERATE POINTS (The Nodes)
    // This adds a "Point Cloud" effect which makes the search volume look solid
    // We only generate dots if performance allows (e.g., < 5000 nodes)
    if (tree.length < 5000) {
      const dotPoints: THREE.Vector3[] = []
      for (let i = 0; i < limit; i++) {
        const node = tree[i]
        if (node) {
          dotPoints.push(this.robot.getTipPosition(node.angles))
        }
      }

      const dotGeo = new THREE.BufferGeometry().setFromPoints(dotPoints)
      const dotMat = new THREE.PointsMaterial({
        color: 0xccffcc, // Slightly paler green for dots
        size: 0.04, // Small nodes
        transparent: true,
        opacity: 0.5,
        sizeAttenuation: true,
      })

      const dots = new THREE.Points(dotGeo, dotMat)
      dots.geometry.setDrawRange(0, 0) // Start hidden
      this.treeMesh.add(dots) // Attach to the line mesh so they move/delete together
    }

    this.scene.add(this.treeMesh)
    this.isAnimatingTree = true
  }

  private animate = () => {
    requestAnimationFrame(this.animate)
    this.controls.update()

    // Target Pulsation Logic (Visual Cue)
    if (this.targetMesh && this.targetMesh.userData.pulsate) {
      this.targetMesh.userData.time += 0.05
      const scale = 1 + Math.sin(this.targetMesh.userData.time) * 0.2
      this.targetMesh.scale.set(scale, scale, scale)
    }

    // Tree Animation Logic
    if (this.isAnimatingTree && this.treeMesh) {
      const linesSpeed = 200 // Vertices per frame
      const currentLineCount = this.treeMesh.geometry.drawRange.count
      const totalLineCount = this.treeMesh.geometry.attributes.position.count

      if (currentLineCount < totalLineCount) {
        this.treeMesh.geometry.setDrawRange(0, currentLineCount + linesSpeed)
      }

      // Animate Dots if they exist
      const dots = this.treeMesh.children.find(
        (c) => c instanceof THREE.Points
      ) as THREE.Points | undefined

      if (dots) {
        const dotsSpeed = 100 // Vertices per frame
        const currentDotCount = dots.geometry.drawRange.count
        const totalDotCount = dots.geometry.attributes.position.count
        if (currentDotCount < totalDotCount) {
          dots.geometry.setDrawRange(0, currentDotCount + dotsSpeed)
        }
      }

      // Completion Check
      if (currentLineCount >= totalLineCount) {
        this.isAnimatingTree = false
        // Ensure fully visible
        this.treeMesh.geometry.setDrawRange(0, Infinity)
        if (dots) dots.geometry.setDrawRange(0, Infinity)

        // START ROBOT MOVEMENT HERE
        if (this.plannedPath.length > 0) {
          this.isPlaying = true
        }
      }
    }

    if (this.isPlaying && this.plannedPath.length > 0) {
      // Speed control: Move 2 steps per frame for smoothness
      if (this.pathIndex < this.plannedPath.length) {
        this.robot.setAngles(this.plannedPath[this.pathIndex])
        this.pathIndex++

        // Update debug lines in real-time if visible
        if (this.collisionDebugGroup.visible) this.updateCollisionDebugLines()
      } else {
        this.isPlaying = false
      }
    }

    this.renderer.render(this.scene, this.camera)
  }

  public resize(width: number, height: number) {
    this.camera.aspect = width / height
    this.camera.updateProjectionMatrix()
    this.renderer.setSize(width, height)
  }

  // Helpers
  private makeGhost(group: THREE.Group) {
    group.traverse((c) => {
      if (c instanceof THREE.Mesh) {
        c.material = new THREE.MeshBasicMaterial({
          color: 0x00ff00,
          wireframe: true,
          transparent: true,
          opacity: 0.3,
        })
      }
    })
  }

  // NEW: Snapshot Standard Tree
  private snapshotStandardTree() {
    if (!this.treeMesh) return

    // Clone the geometry to freeze it
    const geometry = this.treeMesh.geometry.clone()

    // Create a Red/Orange material for the "Ghost" (bad path)
    const material = new THREE.LineBasicMaterial({
      color: 0xff5555, // Red/Orange
      transparent: true,
      opacity: 0.5, // Increased from 0.2 to 0.5 for better visibility
      depthWrite: false,
    })

    this.ghostTreeMesh = new THREE.LineSegments(geometry, material)

    // Ensure it's fully visible (override animation draw range)
    this.ghostTreeMesh.geometry.setDrawRange(0, Infinity)

    this.scene.add(this.ghostTreeMesh)
  }

  private clearGhostTree() {
    if (this.ghostTreeMesh) {
      this.scene.remove(this.ghostTreeMesh)
      this.ghostTreeMesh.geometry.dispose()
      if (Array.isArray(this.ghostTreeMesh.material)) {
        this.ghostTreeMesh.material.forEach((m) => m.dispose())
      } else {
        this.ghostTreeMesh.material.dispose()
      }
      this.ghostTreeMesh = null
    }
  }

  // Toggle visibility of the ghost tree (Standard RRT comparison)
  public setGhostTreeVisible(visible: boolean) {
    if (this.ghostTreeMesh) {
      this.ghostTreeMesh.visible = visible
    }
  }
}
