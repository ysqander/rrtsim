import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { TransformControls } from 'three/addons/controls/TransformControls.js'
import { Robot } from './Robot.ts'
import { RRTPlanner, type RRTParams } from './RRTPlanner.ts'

export type PlannerStats = {
  time: number
  nodes: number
  success: boolean
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
  private transformControl: TransformControls

  // State
  private plannedPath: number[][] = []
  private pathIndex = 0
  private isPlaying = false
  private collisionDebugGroup: THREE.Group

  private algorithm: 'greedy' | 'rrt' = 'rrt'
  private rrtParams: RRTParams = {
    stepSize: 0.05,
    maxIter: 20000,
    goalBias: 0.05,
  }

  // Callbacks to update React UI
  public onStatsUpdate: ((stats: PlannerStats) => void) | null = null

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
    this.camera.position.set(3, 3, 5)
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

    // Obstacle
    this.obstacle = new THREE.Mesh(
      new THREE.BoxGeometry(1, 2, 0.5),
      new THREE.MeshStandardMaterial({ color: 0x888888 })
    )
    this.obstacle.position.set(0.5, 1, 1)
    this.scene.add(this.obstacle)
    this.scene.updateMatrixWorld() // Important for math

    this.planner = new RRTPlanner(this.robot, this.obstacle)

    // 3. Controls
    this.controls = new OrbitControls(this.camera, canvas)
    this.transformControl = new TransformControls(this.camera, canvas)
    this.transformControl.addEventListener('dragging-changed', (event) => {
      this.controls.enabled = !event.value
      // On Drag End -> Plan
      if (!event.value) this.runPlanner()
    })

    // Live Ghost Update while dragging - ONLY in RRT mode if desired, or Greedy mode live update
    this.transformControl.addEventListener('change', () => {
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

  // --- NEW: DYNAMIC JOINTS ---
  public addJoint() {
    this.robot.addJoint()
    this.ghostRobot.addJoint()

    // Re-apply ghost material
    this.makeGhost(this.ghostRobot.sceneObject)

    // Re-run planner if auto-update is desired, or just reset
    this.runPlanner()
  }

  public removeJoint() {
    this.robot.removeJoint()
    this.ghostRobot.removeJoint()

    // Re-apply ghost material
    this.makeGhost(this.ghostRobot.sceneObject)

    // Force scene graph update immediately
    this.scene.updateMatrixWorld(true)

    // Re-run planner if auto-update is desired, or just reset
    this.runPlanner()
  }

  public resetJoints() {
    this.robot.resetConfig()
    this.ghostRobot.resetConfig()
    this.makeGhost(this.ghostRobot.sceneObject)
    this.runPlanner()
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
    this.runPlanner() // Auto-run for effect
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
    this.runPlanner()
  }

  public setAlgorithm(mode: 'greedy' | 'rrt', reset: boolean = false) {
    this.algorithm = mode
    // Hide ghost in both modes as per new plan
    this.ghostRobot.sceneObject.visible = false

    // Clear trees when switching
    if (this.treeMesh) {
      this.scene.remove(this.treeMesh)
      this.treeMesh.geometry.dispose()
      this.treeMesh = null
    }

    if (reset) {
      this.robot.setAngles([0, 0, 0, 0, 0]) // Reset to home
      // Note: We do NOT reset the target position, so it stays in the "stuck" spot
    }

    // If switching to RRT, we want to plan immediately from the current (or reset) state to the target
    if (mode === 'rrt') {
      // IMPORTANT: Force a re-render of the scene state before planning to ensure matrices are updated
      this.scene.updateMatrixWorld(true)
      this.runPlanner()
    }
  }

  public updateRRTParams(params: RRTParams) {
    this.rrtParams = params
    if (this.algorithm === 'rrt') {
      this.runPlanner()
    }
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
    const ARM_THICKNESS = 0.3

    // Define the links we check collision for:
    // Iterate through all moving joints starting from Shoulder (index 2)
    // to the Tip.
    const segments = []
    for (let i = 2; i < joints.length - 1; i++) {
      segments.push({ start: joints[i], end: joints[i + 1] })
    }

    segments.forEach((seg) => {
      // Create a Cylinder to represent the arm volume
      const distance = seg.start.distanceTo(seg.end)
      const geometry = new THREE.CylinderGeometry(
        ARM_THICKNESS / 2,
        ARM_THICKNESS / 2,
        distance,
        8
      )
      geometry.rotateX(Math.PI / 2) // Align with Z initially for LookAt logic

      const material = new THREE.MeshBasicMaterial({
        color: 0xff00ff,
        wireframe: true,
        transparent: true,
        opacity: 0.3,
      })

      const mesh = new THREE.Mesh(geometry, material)

      // Math to position cylinder between two points
      const midPoint = new THREE.Vector3()
        .addVectors(seg.start, seg.end)
        .multiplyScalar(0.5)
      mesh.position.copy(midPoint)
      mesh.lookAt(seg.end)

      this.collisionDebugGroup.add(mesh)
    })

    // Draw the Obstacle Box (Yellow Wireframe)
    const box = new THREE.Box3().setFromObject(this.obstacle)
    box.expandByScalar(0.2) // Visualizing the padding
    const helper = new THREE.Box3Helper(box, 0xffff00)
    this.collisionDebugGroup.add(helper)
  }

  public runPlanner() {
    console.log('Planning...')
    const start = performance.now()
    const startAngles = this.robot.getCurrentAngles()

    // Clear old visual
    if (this.treeMesh) {
      this.scene.remove(this.treeMesh)
      this.treeMesh.geometry.dispose()
      this.treeMesh = null
    }

    if (this.algorithm === 'greedy') {
      // GREEDY IK MODE
      // Just calculate IK and move there immediately
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

    // RRT MODE
    // Ensure robot color is reset
    this.robot.setOverrideColor(null)

    const path = this.planner.plan(
      startAngles,
      this.targetMesh.position,
      this.rrtParams
    )
    const end = performance.now()

    // 1. Visualize Tree
    this.visualizeSearchTree()

    // 2. Report Stats to React
    if (this.onStatsUpdate) {
      this.onStatsUpdate({
        time: Math.round(end - start),
        nodes: this.planner.lastTrees.length,
        success: !!path,
      })
    }

    // 3. Execute Path
    if (path) {
      this.plannedPath = path
      this.pathIndex = 0
      this.isPlaying = true
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
      points.push(this.robot.getTipPosition(node.parent.angles))
      points.push(this.robot.getTipPosition(node.angles))
    }

    const lineGeo = new THREE.BufferGeometry().setFromPoints(points)
    const lineMat = new THREE.LineBasicMaterial({
      color: 0x00ff00, // Matrix Green
      transparent: true,
      opacity: 0.15, // Low opacity makes dense areas "glow" brighter
      depthWrite: false, // Better transparency sorting
    })

    this.treeMesh = new THREE.LineSegments(lineGeo, lineMat)

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
      this.treeMesh.add(dots) // Attach to the line mesh so they move/delete together
    }

    this.scene.add(this.treeMesh)
  }

  private animate = () => {
    requestAnimationFrame(this.animate)
    this.controls.update()

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
}
