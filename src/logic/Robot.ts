import * as THREE from 'three'

// The "DNA" of the robot.
// Equivalent to a simplified URDF or DH Table.
// This describes the relationship of each bone relative to its parent.
interface LinkConfig {
  name: string
  type: 'fixed' | 'revolute' // Fixed = Base/EndEffector, Revolute = Moving Joint
  axis?: 'y' | 'z' // Axis of rotation
  offset: [number, number, number] // Position relative to parent [x, y, z]
  visualLength?: number // For drawing the box/cylinder
  color?: number // Visual color
  limit?: [number, number] // [Min, Max] angles in Radians
}

export class Robot {
  public sceneObject: THREE.Group

  // Dynamic arrays to hold our robot state
  // Instead of this.joint1, we use this.joints[1]
  private joints: THREE.Group[] = []
  public endEffector!: THREE.Mesh

  // Constants for visual and collision
  public readonly JOINT_RADIUS = 0.4
  public readonly ARM_WIDTH = 0.3
  // private readonly COLLISION_MARGIN = 0.05 // Deprecated in favor of local effectiveMargin

  // JOINT parameters
  private CONFIG: LinkConfig[] = [
    {
      name: 'Base',
      type: 'fixed',
      offset: [0, 0.25, 0],
      visualLength: 0.5,
      color: 0x333333,
    },
    // Waist: Full 360 allowed (-PI to PI)
    {
      name: 'Waist',
      type: 'revolute',
      axis: 'y',
      offset: [0, 0.25, 0],
      visualLength: 0.5,
      color: 0x333333,
      limit: [-3.14, 3.14],
    },
    // Shoulder: Can't bend through floor. -90 deg to +90 deg
    {
      name: 'Shoulder',
      type: 'revolute',
      axis: 'z',
      offset: [0, 0.5, 0],
      visualLength: 1.5,
      color: 0x3498db,
      limit: [-1.5, 1.5],
    },
    // Elbow: 0 to 150 degrees
    {
      name: 'Elbow',
      type: 'revolute',
      axis: 'z',
      offset: [0, 1.5, 0],
      visualLength: 1.5,
      color: 0x3498db,
      limit: [-0.1, 2.6],
    },
    // Wrist: -90 to +90
    {
      name: 'Wrist',
      type: 'revolute',
      axis: 'z',
      offset: [0, 1.5, 0],
      visualLength: 1.0,
      color: 0xe67e22,
      limit: [-1.5, 1.5],
    },
    {
      name: 'Tip',
      type: 'fixed',
      offset: [0, 1.0, 0],
      visualLength: 0.2,
      color: 0xffff00,
    },
  ]

  constructor() {
    this.sceneObject = new THREE.Group()
    this.initRobot()
  }

  // --- GETTERS (So the Planner doesn't need to know internal logic) ---

  // How many moving joints?
  public getDoF(): number {
    return this.joints.length
  }

  public getCurrentAngles(): number[] {
    return this.joints.map((j) =>
      j.userData.axis === 'y' ? j.rotation.y : j.rotation.z
    )
  }

  public getLimits(): [number, number][] {
    const limits: [number, number][] = []
    this.CONFIG.forEach((cfg) => {
      if (cfg.type === 'revolute') {
        // Default to full rotation if no limit specified
        limits.push(cfg.limit ? cfg.limit : [-3.14, 3.14])
      }
    })
    return limits
  }

  // --- INITIALIZATION (Dynamic Builder) ---
  public addJoint() {
    // Insert a new joint before the "Tip" (last element)
    const newJointIndex = this.CONFIG.length - 1

    // Get Parent (the one before the new joint)
    const parentConfig = this.CONFIG[newJointIndex - 1]
    // Default visual length for parent if not defined (should be defined)
    const parentLength = parentConfig?.visualLength || 1.0

    const newJoint: LinkConfig = {
      name: `Joint ${newJointIndex}`,
      type: 'revolute',
      axis: Math.random() > 0.5 ? 'z' : 'y',
      // IMPORTANT: Offset is determined by PARENT'S length
      offset: [0, parentLength, 0],
      visualLength: 1.0, // Standard length for new joints
      color: Math.random() * 0xffffff,
      limit: [-2.0, 2.0],
    }

    // Insert before Tip
    this.CONFIG.splice(newJointIndex, 0, newJoint)

    // Update Tip's offset to match the NEW joint's length
    const tipIndex = this.CONFIG.length - 1
    if (this.CONFIG[tipIndex]) {
      this.CONFIG[tipIndex]!.offset = [0, newJoint.visualLength!, 0]
    }

    this.rebuild()
  }

  public removeJoint() {
    // Ensure minimum joints (e.g., 3: Base, Waist, Tip)
    if (this.CONFIG.length <= 4) return

    // Remove the joint before Tip.
    const removeIndex = this.CONFIG.length - 2

    // Sanity check
    if (removeIndex > 1) {
      // Identify Parent of the removed joint
      const parentConfig = this.CONFIG[removeIndex - 1]
      const parentLength = parentConfig?.visualLength || 1.0

      // Remove the configuration entry
      this.CONFIG.splice(removeIndex, 1)

      // Update the NEW joint at this index (Tip) to use offset matching the Parent
      if (this.CONFIG[removeIndex]) {
        this.CONFIG[removeIndex]!.offset = [0, parentLength, 0]
      }

      this.rebuild()
    }
  }

  public resetConfig() {
    // Reset to default config
    this.CONFIG = [
      {
        name: 'Base',
        type: 'fixed',
        offset: [0, 0.25, 0],
        visualLength: 0.5,
        color: 0x333333,
      },
      {
        name: 'Waist',
        type: 'revolute',
        axis: 'y',
        offset: [0, 0.25, 0],
        visualLength: 0.5,
        color: 0x333333,
        limit: [-3.14, 3.14],
      },
      {
        name: 'Shoulder',
        type: 'revolute',
        axis: 'z',
        offset: [0, 0.5, 0],
        visualLength: 1.5,
        color: 0x3498db,
        limit: [-1.5, 1.5],
      },
      {
        name: 'Elbow',
        type: 'revolute',
        axis: 'z',
        offset: [0, 1.5, 0],
        visualLength: 1.5,
        color: 0x3498db,
        limit: [-0.1, 2.6],
      },
      {
        name: 'Wrist',
        type: 'revolute',
        axis: 'z',
        offset: [0, 1.5, 0],
        visualLength: 1.0,
        color: 0xe67e22,
        limit: [-1.5, 1.5],
      },
      {
        name: 'Tip',
        type: 'fixed',
        offset: [0, 1.0, 0],
        visualLength: 0.2,
        color: 0xffff00,
      },
    ]
    this.rebuild()
  }

  private rebuild() {
    // Clear existing meshes
    this.sceneObject.clear()
    this.joints = []
    this.initRobot()
  }

  private initRobot() {
    const matJoint = new THREE.MeshStandardMaterial({ color: 0xe74c3c }) // Red Ball

    let parentGroup = this.sceneObject

    this.CONFIG.forEach((cfg, index) => {
      // 1. Create the Pivot Group
      const linkGroup = new THREE.Group()
      linkGroup.position.set(...cfg.offset)

      // Tag the group so we know how to rotate it later
      linkGroup.userData = {
        isJoint: cfg.type === 'revolute',
        axis: cfg.axis,
        configIndex: index,
      }

      // Add to hierarchy
      parentGroup.add(linkGroup)
      parentGroup = linkGroup // Set as parent for next link

      // Store reference if it's a moving joint
      if (cfg.type === 'revolute') {
        this.joints.push(linkGroup)

        // Add a visual "Joint Sphere" (The red hinge)
        const jointMesh = new THREE.Mesh(
          new THREE.SphereGeometry(this.JOINT_RADIUS),
          matJoint
        )
        linkGroup.add(jointMesh)
      }

      // 2. Create the Visual Bone (The Box/Cylinder)
      if (cfg.visualLength) {
        const material = new THREE.MeshStandardMaterial({ color: cfg.color })
        let mesh: THREE.Mesh

        if (cfg.name === 'Base' || cfg.name === 'Waist') {
          mesh = new THREE.Mesh(
            new THREE.CylinderGeometry(0.5, 0.5, cfg.visualLength),
            material
          )
        } else if (cfg.name === 'Tip') {
          this.endEffector = new THREE.Mesh(
            new THREE.SphereGeometry(0.2),
            material
          )
          mesh = this.endEffector // Track the tip
        } else {
          mesh = new THREE.Mesh(
            new THREE.BoxGeometry(
              this.ARM_WIDTH,
              cfg.visualLength,
              this.ARM_WIDTH
            ),
            material
          )
        }

        // Offset mesh so it sits "on top" of the pivot
        mesh.position.y = cfg.visualLength / 2
        linkGroup.add(mesh)
      }
    })
  }

  // --- NEW: Color Override for Visualization ---
  public setOverrideColor(color: number | null) {
    this.sceneObject.traverse((c) => {
      if (c instanceof THREE.Mesh) {
        const mat = c.material as THREE.MeshStandardMaterial
        // We only want to override the "Body" parts, not necessarily the red joints?
        // Actually user said "turn red". Let's turn everything red.

        // Save original if not saved
        if (mat.userData.originalColor === undefined) {
          mat.userData.originalColor = mat.color.getHex()
        }

        if (color !== null) {
          mat.color.setHex(color)
        } else {
          mat.color.setHex(mat.userData.originalColor)
        }
      }
    })
  }

  // --- GENERIC FK SOLVER ---
  // Calculates the position of every link given a set of angles
  private computeFK(angles: number[]) {
    let currentGlobal = new THREE.Matrix4() // Start at Identity (0,0,0)

    const matrices: THREE.Matrix4[] = []
    let angleIndex = 0

    this.CONFIG.forEach((cfg) => {
      // 1. Translation (From parent to this joint)
      const translation = new THREE.Matrix4().makeTranslation(...cfg.offset)

      // 2. Rotation (If it moves)
      const rotation = new THREE.Matrix4()
      if (cfg.type === 'revolute') {
        const angle = angles[angleIndex] || 0
        if (cfg.axis === 'y') rotation.makeRotationY(angle)
        if (cfg.axis === 'z') rotation.makeRotationZ(angle)
        angleIndex++
      }

      // 3. Local Transform = Translation * Rotation
      const local = translation.multiply(rotation)

      // 4. Global Transform = ParentGlobal * Local
      currentGlobal = currentGlobal.clone().multiply(local)

      // Save it (We assume 1 matrix per config entry)
      matrices.push(currentGlobal.clone())
    })

    return matrices
  }

  // --- GENERIC IK SOLVER (CCD) ---
  public calculateIK(
    targetPos: THREE.Vector3,
    initialAngles?: number[]
  ): number[] {
    const angles = initialAngles ? [...initialAngles] : this.getCurrentAngles()
    const iterations = 15
    const threshold = 0.01

    // Loop Iterations
    for (let iter = 0; iter < iterations; iter++) {
      // Iterate BACKWARDS through joints (Tip -> Base)
      for (let j = this.joints.length - 1; j >= 0; j--) {
        // A. Compute FK to find where all the joints are currently
        const fkMatrices = this.computeFK(angles)

        // The last matrix in fkMatrices corresponds to the EndEffector (Tip)
        // But we need to map joint index 'j' to the CONFIG index.
        // Since Config has "Base" (fixed) at 0, Joint 0 (Waist) is Config[1].
        // A safer way is to grab the matrix matching the EndEffector config index.
        const tipMatrix = fkMatrices[this.CONFIG.length - 1]
        const currentTipPos = new THREE.Vector3().setFromMatrixPosition(
          tipMatrix!
        )

        // If the tip is close to target, we don't need to calculate the necessary angle rotation delta
        if (currentTipPos.distanceTo(targetPos) < threshold) return angles

        // Get the joint matrix which includes position coordinates, roation and scale.
        const joint = this.joints[j]
        const configIndex = joint!.userData.configIndex
        const jointMatrix = fkMatrices[configIndex]
        const jointAxis = joint!.userData.axis

        // B. Convert from World coordinates -> to local joint coordinates
        const invJointMatrix = jointMatrix!.clone().invert()
        const localTip = currentTipPos.clone().applyMatrix4(invJointMatrix)
        const localTarget = targetPos.clone().applyMatrix4(invJointMatrix)

        // C. Calculate Delta
        let delta = 0
        if (jointAxis === 'y') {
          const aTip = Math.atan2(localTip.x, localTip.z)
          const aTgt = Math.atan2(localTarget.x, localTarget.z)
          delta = aTgt - aTip
        } else {
          // Z axis
          const aTip = Math.atan2(localTip.y, localTip.x)
          const aTgt = Math.atan2(localTarget.y, localTarget.x)
          delta = aTgt - aTip
        }

        // Normalize Delta to shortest path (-PI to PI)
        while (delta > Math.PI) delta -= 2 * Math.PI
        while (delta < -Math.PI) delta += 2 * Math.PI

        // D. Apply
        angles[j]! += delta

        // --- FIX: CLAMP TO LIMITS ---
        const limit = this.CONFIG[configIndex]?.limit
        if (limit) {
          angles[j] = Math.max(limit[0], Math.min(limit[1], angles[j]!))
        }
      }
    }
    return angles
  }

  // --- ROBUST IK SOLVER (CCD + Random Restarts) ---
  // Tries to find a solution that is also collision-free AND reaches the target
  public solveRobustIK(
    targetPos: THREE.Vector3,
    obstacle: THREE.Mesh
  ): number[] {
    const limits = this.getLimits()
    const DISTANCE_THRESHOLD = 0.1 // Must be this close to be valid

    // Helper to check validity (Collision Free + Reaches Target)
    const isValid = (angles: number[]) => {
      if (this.checkCollision(angles, obstacle)) return false
      const tip = this.getTipPosition(angles)
      return tip.distanceTo(targetPos) < DISTANCE_THRESHOLD
    }

    // 1. Try Standard Greedy IK first
    const standardSolution = this.calculateIK(targetPos)
    if (isValid(standardSolution)) {
      return standardSolution
    }

    // 2. If collision or unreachable, try Random initial angle retries.
    // (We prefer a valid solution even if it takes a few ms more)
    for (let k = 0; k < 100; k++) {
      // Generate random seed
      const randomSeed = limits.map((l) => Math.random() * (l[1] - l[0]) + l[0])
      const attempt = this.calculateIK(targetPos, randomSeed)
      if (isValid(attempt)) {
        return attempt
      }
    }

    // 3. If all else fails, return the standard one (at least visualizes "something")
    // Note: This might be a "bad" solution (collision or far away), but it's the best visual we have.
    return standardSolution
  }

  // Generic Visual Update
  public setAngles(angles: number[]) {
    this.joints.forEach((joint, i) => {
      if (joint.userData.axis === 'y') joint.rotation.y = angles[i]!
      else joint.rotation.z = angles[i]!
    })
  }

  // --- COLLISION CHECKER ---
  // NEW HELPER: Get positions of all joints in the chain
  public getJointPositions(angles: number[]) {
    const matrices = this.computeFK(angles)

    // Map matrix to Vector3
    return matrices.map((m) => new THREE.Vector3().setFromMatrixPosition(m))
  }

  // OPTIMIZED COLLISION CHECK
  // Uses Line Segment Sampling instead of Mesh Bounding Boxes
  public checkCollision(angles: number[], obstacle: THREE.Mesh): boolean {
    // 1. Get the Obstacle's Box ONCE (Fast)
    // In a real app, cache this. Don't compute it every frame.
    // Since the wall doesn't move, we assume its World Matrix is up to date.
    const obstacleBox = new THREE.Box3().setFromObject(obstacle)

    // Reset scalar expansion (no extra fatness, trust the spheres)
    obstacleBox.expandByScalar(0.01)

    // 2. Get Virtual Skeleton Positions (Pure Math, No Visual Updates)
    // [Base, Waist, Shoulder, Elbow, Wrist, Tip]
    // Note: These positions correspond 1-to-1 with the CONFIG array
    const joints = this.getJointPositions(angles)

    // 3. Check Sampling Points along the bones
    // We iterate through the CONFIG to find segments (where visualLength > 0)
    const samplesPerLink = 10
    // INCREASED MARGIN: To better match the visual debugging and avoid clipping
    // Was 0.05, increasing to 0.15 to provide a safer buffer around the robot
    const effectiveMargin = 0.15
    const collisionRadius = this.ARM_WIDTH / 2 + effectiveMargin
    const jointRadius = this.JOINT_RADIUS + effectiveMargin

    // FIX: Iterate 0 to length-1. Check CONFIG[i] mesh on segment i -> i+1
    for (let i = 0; i < joints.length - 1; i++) {
      // Only check segments that have visual length defined in the configuration
      if (this.CONFIG[i]?.visualLength) {
        const start = joints[i]
        const end = joints[i + 1]

        if (start && end) {
          for (let k = 0; k <= samplesPerLink; k++) {
            const t = k / samplesPerLink
            const point = new THREE.Vector3().lerpVectors(start, end, t)
            const sphere = new THREE.Sphere(point, collisionRadius)

            if (obstacleBox.intersectsSphere(sphere)) {
              return true // HIT!
            }
          }
        }
      }
    }

    // 4. CHECK JOINT SPHERES DYNAMICALLY
    // We iterate through CONFIG to find 'revolute' joints that have large hubs
    // FIX: Also check fixed joints if they have geometry (like Tip)
    this.CONFIG.forEach((cfg, index) => {
      // Check revolute joints OR the Tip (last one)
      if (cfg.type === 'revolute' || index === this.CONFIG.length - 1) {
        const jointPos = joints[index]
        if (jointPos) {
          const sphere = new THREE.Sphere(jointPos, jointRadius)
          if (obstacleBox.intersectsSphere(sphere)) {
            return true // HIT!
          }
        }
      }
    })

    return false
  }

  // --- HELPER FOR VISUALIZATION ---
  // Calculates the 3D position of the tip for a given set of angles
  // without moving the actual robot.
  public getTipPosition(angles: number[]): THREE.Vector3 {
    const chain = this.computeFK(angles)
    // The last matrix in the chain is the End Effector
    // (Based on our CONFIG, the last entry is the Tip)
    const tipMatrix = chain[chain.length - 1]
    return new THREE.Vector3().setFromMatrixPosition(tipMatrix!)
  }
}
