import * as THREE from 'three'

// The "DNA" of the robot.
// Equivalent to a simplified URDF or DH Table.
// This describes the relationship of each bone relative to its parent.
export interface LinkConfig {
  name: string
  type: 'fixed' | 'revolute' // Fixed = Base/EndEffector, Revolute = Moving Joint
  axis?: 'y' | 'z' // Axis of rotation
  offset: [number, number, number] // Position relative to parent [x, y, z]
  visualLength?: number // For drawing the box/cylinder
  color?: number // Visual color
  limit?: [number, number] // [Min, Max] angles in Radians
}

// Oriented Bounding Box for accurate rotated obstacle collision detection
export type OBBObstacle = {
  halfSize: [number, number, number] // half-widths in local box space
  matrix: number[] // 16-length world matrix (row-major from Matrix4.toArray())
}

// Default robot configuration (9 joints for better flexibility)
const DEFAULT_CONFIG: LinkConfig[] = [
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
  // Joint 5
  {
    name: 'Joint 5',
    type: 'revolute',
    axis: 'z',
    offset: [0, 1.0, 0],
    visualLength: 1.0,
    color: 0x9b59b6,
    limit: [-2.0, 2.0],
  },
  // Joint 6
  {
    name: 'Joint 6',
    type: 'revolute',
    axis: 'y',
    offset: [0, 1.0, 0],
    visualLength: 1.0,
    color: 0x1abc9c,
    limit: [-2.0, 2.0],
  },
  // Joint 7
  {
    name: 'Joint 7',
    type: 'revolute',
    axis: 'z',
    offset: [0, 1.0, 0],
    visualLength: 1.0,
    color: 0xe91e63,
    limit: [-2.0, 2.0],
  },
  // Joint 8
  {
    name: 'Joint 8',
    type: 'revolute',
    axis: 'y',
    offset: [0, 1.0, 0],
    visualLength: 1.0,
    color: 0x00bcd4,
    limit: [-2.0, 2.0],
  },
  // Joint 9
  {
    name: 'Joint 9',
    type: 'revolute',
    axis: 'z',
    offset: [0, 1.0, 0],
    visualLength: 1.0,
    color: 0xff5722,
    limit: [-2.0, 2.0],
  },
  {
    name: 'Tip',
    type: 'fixed',
    offset: [0, 1.0, 0],
    visualLength: 0.2,
    color: 0xffff00,
  },
]

export class Robot {
  public sceneObject: THREE.Group

  // Dynamic arrays to hold our robot state
  // Instead of this.joint1, we use this.joints[1]
  private joints: THREE.Group[] = []
  public endEffector!: THREE.Mesh

  // Base constants for visual and collision (before scaling)
  private readonly BASE_JOINT_RADIUS = 0.4
  private readonly BASE_ARM_WIDTH = 0.3
  private readonly BASE_TIP_RADIUS = 0.2

  // Arm length scale factor (0.7 = default for thinner robot, 0.5 = half length, 2.0 = double length)
  private armScale: number = 0.7

  // Joint/arm thickness scale factor (0.3 = default for thin robot)
  private jointScale: number = 0.3

  // Tip (end effector) scale factor (0.6 = default to match joint size)
  private tipScale: number = 0.6

  // Getters for scaled dimensions
  public get JOINT_RADIUS(): number {
    return this.BASE_JOINT_RADIUS * this.jointScale
  }

  public get ARM_WIDTH(): number {
    return this.BASE_ARM_WIDTH * this.jointScale
  }

  public get TIP_RADIUS(): number {
    return this.BASE_TIP_RADIUS * this.tipScale
  }

  // ==========================================================================
  // COLLISION DETECTION PARAMETERS
  // ==========================================================================
  // These constants control how collision detection works. We use DIFFERENT
  // margins for obstacle vs self-collision because they serve different purposes:
  //
  // SAMPLES_PER_SEGMENT: More samples = more accurate but slower.
  //   10 samples means we check every ~10% of each link segment.
  //   Used for OBSTACLE collision only (self-collision uses analytic distance).
  //
  // OBSTACLE_COLLISION_MARGIN: Extra "padding" when checking against walls/obstacles.
  //   0.15 provides a safe buffer to prevent clipping and near-misses with external
  //   objects. We want the robot to stay well clear of obstacles.
  //
  // SELF_COLLISION_MARGIN: Smaller margin for checking if arm segments intersect.
  //   0.02 is a small tolerance to account for numerical precision.
  //   We use a SMALLER margin here because adjacent segments on the same robot
  //   are naturally close together at joints. A larger margin would cause
  //   false positives where valid configurations are rejected.
  //
  // SELF_COLLISION_TRIM: Fixed distance to trim from each segment endpoint
  //   before checking self-collision. This prevents false positives near joints
  //   where segments naturally meet. 0.15 units provides good clearance without
  //   missing real collisions in the middle of segments.
  // ==========================================================================
  private readonly SAMPLES_PER_SEGMENT = 10
  private readonly OBSTACLE_COLLISION_MARGIN = 0.15
  private readonly SELF_COLLISION_MARGIN = 0.02
  private readonly SELF_COLLISION_TRIM = 0.15

  // JOINT parameters - public for serialization to worker
  // Initialized from DEFAULT_CONFIG (deep copy to avoid mutation)
  public CONFIG: LinkConfig[] = JSON.parse(JSON.stringify(DEFAULT_CONFIG))

  constructor(config?: LinkConfig[]) {
    this.sceneObject = new THREE.Group()
    if (config) {
      this.CONFIG = config
    }
    // Apply default arm scale to CONFIG before building visuals
    if (this.armScale !== 1.0) {
      this.applyArmScale()
    }
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

    // Apply arm scale to new joint's length
    const newJointLength = 1.0 * this.armScale

    const newJoint: LinkConfig = {
      name: `Joint ${newJointIndex}`,
      type: 'revolute',
      axis: Math.random() > 0.5 ? 'z' : 'y',
      // IMPORTANT: Offset is determined by PARENT'S length
      offset: [0, parentLength, 0],
      visualLength: newJointLength, // Scaled length for new joints
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
    // Reset to default config (deep copy to avoid mutation)
    this.CONFIG = JSON.parse(JSON.stringify(DEFAULT_CONFIG))
    // Re-apply current arm scale after reset
    if (this.armScale !== 1.0) {
      this.applyArmScale()
    }
    // Joint scale is preserved through rebuild() automatically via getters
    this.rebuild()
  }

  /**
   * Set the arm length scale factor.
   * @param scale Scale factor (1.0 = default, 0.5 = half length, 2.0 = double length)
   */
  public setArmLengthScale(scale: number) {
    this.armScale = Math.max(0.3, Math.min(2.5, scale)) // Clamp to safe range
    this.applyArmScale()
    this.rebuild()
  }

  /**
   * Get the current arm length scale factor.
   */
  public getArmLengthScale(): number {
    return this.armScale
  }

  /**
   * Set the joint/arm thickness scale factor.
   * @param scale Scale factor (1.0 = default, 0.6 = thinner, 0.2 = very thin)
   */
  public setJointScale(scale: number) {
    this.jointScale = Math.max(0.1, Math.min(2.0, scale)) // Clamp to safe range
    this.rebuild()
  }

  /**
   * Get the current joint/arm thickness scale factor.
   */
  public getJointScale(): number {
    return this.jointScale
  }

  /**
   * Set the tip (end effector) scale factor.
   * @param scale Scale factor (1.0 = default, 0.1 = very small, 2.0 = large)
   */
  public setTipScale(scale: number) {
    this.tipScale = Math.max(0.05, Math.min(3.0, scale)) // Clamp to safe range
    this.rebuild()
  }

  /**
   * Get the current tip (end effector) scale factor.
   */
  public getTipScale(): number {
    return this.tipScale
  }

  /**
   * Apply the arm scale to the CONFIG.
   * Scales the visualLength of arm segments and updates offsets accordingly.
   */
  private applyArmScale() {
    // Base lengths for each segment type (from DEFAULT_CONFIG)
    const baseLengths: Record<string, number> = {
      Base: 0.5,
      Waist: 0.5,
      Shoulder: 1.5,
      Elbow: 1.5,
      Wrist: 1.0,
      'Joint 5': 1.0,
      'Joint 6': 1.0,
      'Joint 7': 1.0,
      'Joint 8': 1.0,
      'Joint 9': 1.0,
      Tip: 0.2,
    }

    // First pass: Update visualLength for scalable segments
    for (let i = 0; i < this.CONFIG.length; i++) {
      const cfg = this.CONFIG[i]!
      const baseLength = baseLengths[cfg.name] ?? 1.0 // Default for dynamically added joints

      // Only scale arm segments (not Base, Waist, or Tip)
      if (
        cfg.name !== 'Base' &&
        cfg.name !== 'Waist' &&
        cfg.name !== 'Tip' &&
        cfg.visualLength !== undefined
      ) {
        cfg.visualLength = baseLength * this.armScale
      }
    }

    // Second pass: Update offsets based on parent's visualLength
    for (let i = 1; i < this.CONFIG.length; i++) {
      const parentCfg = this.CONFIG[i - 1]!
      const cfg = this.CONFIG[i]!

      // Offset Y should match parent's visual length
      if (parentCfg.visualLength !== undefined) {
        cfg.offset = [0, parentCfg.visualLength, 0]
      }
    }
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
            new THREE.SphereGeometry(this.TIP_RADIUS),
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
  // Calculates the position of every joint given a set of angles
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

  // --- GENERIC IK SOLVER (CCD)  used in the Greedy approach---
  // In simple terms, it is as if each joint in the robot arm, "looks" at the current position of the tip of the robot and the target position and decides to rotate itself to minimize tht distance.
  // By looping through each joint, and doing this wiggle adjustments, we get closer and closer and in simple reachable cases, the end point gets to the target. It's counterintuitive that this simple approach works but it does.

  public calculateIK(
    targetPos: THREE.Vector3,
    initialAngles?: number[]
  ): number[] {
    const angles = initialAngles ? [...initialAngles] : this.getCurrentAngles() // this gets the initial angles of every joint in the robot
    const iterations = 15
    const threshold = 0.01

    // Loop Iterations
    for (let iter = 0; iter < iterations; iter++) {
      // Iterate BACKWARDS through joints (Tip -> Base)
      for (let j = this.joints.length - 1; j >= 0; j--) {
        // A. Compute the position of every joint relative to the origin (0,0,0) also called Wolrd Space
        const fkMatrices = this.computeFK(angles)

        const tipMatrix = fkMatrices[this.CONFIG.length - 1] // we grab the last matrix whic is the tip or end-effector (whatever would be doing the action)
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
  // Accepts Mesh, Box3, OBB, or arrays of any combination
  // Optional `random` parameter allows passing a seeded PRNG for deterministic behavior
  public solveRobustIK(
    targetPos: THREE.Vector3,
    obstacle:
      | THREE.Mesh
      | THREE.Box3
      | { halfSize: [number, number, number]; matrix: number[] }
      | Array<
          | THREE.Mesh
          | THREE.Box3
          | { halfSize: [number, number, number]; matrix: number[] }
        >,
    random: () => number = Math.random
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
    // Uses the provided random function for deterministic behavior when seeded
    for (let k = 0; k < 100; k++) {
      // Generate random seed using the provided random function
      const randomSeed = limits.map((l) => random() * (l[1] - l[0]) + l[0])
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

  // --- HELPER FOR COLLISION CHECK (below) ---
  public getJointPositions(angles: number[]) {
    const matrices = this.computeFK(angles)

    // Map matrix to Vector3
    return matrices.map((m) => new THREE.Vector3().setFromMatrixPosition(m))
  }

  /**
   * Samples evenly-spaced points along a line segment.
   *
   * This is a shared helper used by both obstacle collision and self-collision
   * detection. By using the same sampling for both, we ensure consistent behavior.
   *
   * @param start - Start point of the segment
   * @param end - End point of the segment
   * @returns Array of Vector3 points along the segment (including start and end)
   */
  private sampleSegmentPoints(
    start: THREE.Vector3,
    end: THREE.Vector3
  ): THREE.Vector3[] {
    const points: THREE.Vector3[] = []

    for (let k = 0; k <= this.SAMPLES_PER_SEGMENT; k++) {
      const t = k / this.SAMPLES_PER_SEGMENT
      // Linear interpolation: point = start + t * (end - start)
      points.push(new THREE.Vector3().lerpVectors(start, end, t))
    }

    return points
  }

  /**
   * Returns the collision radius used for link segments when checking OBSTACLES.
   * Uses the larger margin for safety buffer against external objects.
   */
  private getObstacleCollisionRadius(): number {
    return this.ARM_WIDTH / 2 + this.OBSTACLE_COLLISION_MARGIN
  }

  /**
   * Returns the collision radius used for joint spheres when checking OBSTACLES.
   * Joints are larger than arm segments, so they get a bigger radius.
   * Uses slightly less conservative margin for joints to fit through narrow gates
   * while keeping link segment margin unchanged.
   */
  private getJointCollisionRadius(): number {
    return this.JOINT_RADIUS + Math.min(0.1, this.OBSTACLE_COLLISION_MARGIN)
  }

  /**
   * Returns the collision radius used for SELF-COLLISION detection.
   * Uses a smaller margin because adjacent segments are naturally close at joints.
   * We only want to detect actual geometric intersection, not "closeness".
   */
  private getSelfCollisionRadius(): number {
    return this.ARM_WIDTH / 2 + this.SELF_COLLISION_MARGIN
  }

  /**
   * Tests if a sphere intersects an Oriented Bounding Box (OBB).
   * Transforms the sphere center to box-local space and performs AABB test there.
   *
   * @param centerWorld - Sphere center in world coordinates
   * @param radius - Sphere radius
   * @param obb - The oriented bounding box with halfSize and world matrix
   * @returns true if there is intersection
   */
  private intersectsSphereOBB(
    centerWorld: THREE.Vector3,
    radius: number,
    obb: { halfSize: [number, number, number]; matrix: number[] }
  ): boolean {
    const mat = new THREE.Matrix4().fromArray(obb.matrix)
    const inv = new THREE.Matrix4().copy(mat).invert()

    // Transform center to the box's local space
    const cLocal = centerWorld.clone().applyMatrix4(inv)
    const hs = obb.halfSize

    // Closest point in the AABB (in local space)
    const qx = Math.max(-hs[0], Math.min(hs[0], cLocal.x))
    const qy = Math.max(-hs[1], Math.min(hs[1], cLocal.y))
    const qz = Math.max(-hs[2], Math.min(hs[2], cLocal.z))

    const dx = cLocal.x - qx
    const dy = cLocal.y - qy
    const dz = cLocal.z - qz
    return dx * dx + dy * dy + dz * dz <= radius * radius
  }

  // ==========================================================================
  // SEGMENT-TO-SEGMENT DISTANCE (Analytic Approach)
  // ==========================================================================
  //
  // Computes the minimum Euclidean distance between two 3D line segments.
  // Based on the parametric approach from Ericson's "Real-Time Collision Detection".
  //
  // Each segment is defined by two endpoints:
  //   Segment 1: p1 -> q1
  //   Segment 2: p2 -> q2
  //
  // We parameterize each segment as:
  //   S1(s) = p1 + s * (q1 - p1),  s in [0, 1]
  //   S2(t) = p2 + t * (q2 - p2),  t in [0, 1]
  //
  // The algorithm finds the (s, t) pair that minimizes ||S1(s) - S2(t)||.
  // ==========================================================================

  /**
   * Computes the closest distance between two 3D line segments.
   *
   * @param p1 - Start of segment 1
   * @param q1 - End of segment 1
   * @param p2 - Start of segment 2
   * @param q2 - End of segment 2
   * @returns The minimum distance between the two segments
   */
  private closestDistanceBetweenSegments(
    p1: THREE.Vector3,
    q1: THREE.Vector3,
    p2: THREE.Vector3,
    q2: THREE.Vector3
  ): number {
    // Direction vectors
    const d1 = new THREE.Vector3().subVectors(q1, p1) // Segment 1 direction
    const d2 = new THREE.Vector3().subVectors(q2, p2) // Segment 2 direction
    const r = new THREE.Vector3().subVectors(p1, p2) // Vector from p2 to p1

    const a = d1.dot(d1) // Squared length of segment 1
    const e = d2.dot(d2) // Squared length of segment 2
    const f = d2.dot(r)

    const EPSILON = 1e-7

    let s: number
    let t: number

    // Check if either or both segments degenerate into points
    if (a <= EPSILON && e <= EPSILON) {
      // Both segments are points
      return p1.distanceTo(p2)
    }

    if (a <= EPSILON) {
      // Segment 1 is a point
      s = 0
      t = Math.max(0, Math.min(1, f / e))
    } else {
      const c = d1.dot(r)
      if (e <= EPSILON) {
        // Segment 2 is a point
        t = 0
        s = Math.max(0, Math.min(1, -c / a))
      } else {
        // General case: neither segment is degenerate
        const b = d1.dot(d2)
        const denom = a * e - b * b // Always >= 0

        // If segments are not parallel, compute closest point on line 1 to line 2
        // and clamp to segment 1. Otherwise pick arbitrary s (here 0).
        if (denom !== 0) {
          s = Math.max(0, Math.min(1, (b * f - c * e) / denom))
        } else {
          s = 0
        }

        // Compute point on line 2 closest to S1(s)
        t = (b * s + f) / e

        // If t is outside [0,1], clamp and recompute s
        if (t < 0) {
          t = 0
          s = Math.max(0, Math.min(1, -c / a))
        } else if (t > 1) {
          t = 1
          s = Math.max(0, Math.min(1, (b - c) / a))
        }
      }
    }

    // Compute the closest points
    const c1 = new THREE.Vector3().addVectors(p1, d1.clone().multiplyScalar(s))
    const c2 = new THREE.Vector3().addVectors(p2, d2.clone().multiplyScalar(t))

    return c1.distanceTo(c2)
  }

  // ==========================================================================
  // SEGMENT TRIMMING HELPER
  // ==========================================================================
  //
  // Shortens a line segment from both ends by a fixed amount.
  // This is used to avoid false-positive self-collisions near joints, where
  // adjacent segments naturally meet and would otherwise appear to "collide".
  //
  // If the trim amount would invert or collapse the segment (i.e., trim more
  // than half the segment length from each end), we return the midpoint as
  // both start and end (a degenerate zero-length segment).
  // ==========================================================================

  /**
   * Trims a segment from both ends by a fixed amount.
   *
   * @param start - Original start point of the segment
   * @param end - Original end point of the segment
   * @param trimAmount - Distance to trim from each end
   * @returns New trimmed segment { start, end }
   */
  private trimSegment(
    start: THREE.Vector3,
    end: THREE.Vector3,
    trimAmount: number
  ): { start: THREE.Vector3; end: THREE.Vector3 } {
    const length = start.distanceTo(end)

    // If trimming would collapse or invert the segment, return midpoint
    if (trimAmount * 2 >= length) {
      const midpoint = new THREE.Vector3().lerpVectors(start, end, 0.5)
      return { start: midpoint.clone(), end: midpoint.clone() }
    }

    // Calculate the trim ratio (how far along the segment to move each endpoint)
    const trimRatio = trimAmount / length

    // New start is moved inward from original start
    const newStart = new THREE.Vector3().lerpVectors(start, end, trimRatio)
    // New end is moved inward from original end
    const newEnd = new THREE.Vector3().lerpVectors(start, end, 1 - trimRatio)

    return { start: newStart, end: newEnd }
  }

  // ==========================================================================
  // COMPLETE COLLISION CHECK (Obstacle + Self-Collision)
  // ==========================================================================
  //
  // This method checks for TWO types of collisions:
  //   1. OBSTACLE COLLISION: Robot hitting external objects (walls, boxes, etc.)
  //   2. SELF-COLLISION: Robot's own arm segments intersecting each other
  //
  // Both checks are essential for valid motion planning. The RRT algorithm
  // calls this method to validate each potential configuration.
  //
  // Uses Line Segment Sampling instead of Mesh Bounding Boxes for accuracy.
  // Supports: Mesh, Box3 (AABB), OBBObstacle (oriented bounding box), or arrays of any.
  // OBB support enables accurate collision detection for rotated obstacles.
  // ==========================================================================
  public checkCollision(
    angles: number[],
    obstacle:
      | THREE.Mesh
      | THREE.Box3
      | { halfSize: [number, number, number]; matrix: number[] }
      | Array<
          | THREE.Mesh
          | THREE.Box3
          | { halfSize: [number, number, number]; matrix: number[] }
        >
  ): boolean {
    // ========================================
    // PART 1: CHECK SELF-COLLISION FIRST
    // ========================================
    if (this.checkSelfCollision(angles)) {
      return true // Self-collision detected - configuration is invalid
    }

    // ========================================
    // PART 2: CHECK OBSTACLE COLLISION
    // ========================================

    // Normalize obstacles into separate arrays for AABBs and OBBs
    const asArray = Array.isArray(obstacle) ? obstacle : [obstacle]
    const boxObstacles: THREE.Box3[] = []
    const obbObstacles: {
      halfSize: [number, number, number]
      matrix: number[]
    }[] = []

    for (const o of asArray) {
      // Check for Box3 (has min/max properties)
      if (
        o &&
        typeof o === 'object' &&
        'min' in o &&
        'max' in o &&
        (o as THREE.Box3).isBox3
      ) {
        const box = (o as THREE.Box3).clone()
        box.expandByScalar(0.01)
        boxObstacles.push(box)
      }
      // Check for OBB (has halfSize and matrix properties)
      else if (o && typeof o === 'object' && 'halfSize' in o && 'matrix' in o) {
        obbObstacles.push(
          o as { halfSize: [number, number, number]; matrix: number[] }
        )
      }
      // Check for Mesh (has isMesh property)
      else if (o && typeof o === 'object' && (o as THREE.Mesh).isMesh) {
        // Convert Mesh to OBB using its geometry and world matrix
        const mesh = o as THREE.Mesh
        const geom = mesh.geometry as THREE.BufferGeometry
        if (!geom.boundingBox) geom.computeBoundingBox()
        const bb = geom.boundingBox!
        obbObstacles.push({
          halfSize: [
            (bb.max.x - bb.min.x) / 2,
            (bb.max.y - bb.min.y) / 2,
            (bb.max.z - bb.min.z) / 2,
          ],
          matrix: mesh.matrixWorld.toArray(),
        })
      }
    }

    // 2. Get Virtual Skeleton Positions
    const joints = this.getJointPositions(angles)

    // 3. Get collision radii
    const segRadius = this.getObstacleCollisionRadius()
    const jointRadius = this.getJointCollisionRadius()

    // 4. Check Sampling Points along the bones vs all obstacles
    for (let i = 0; i < joints.length - 1; i++) {
      if (this.CONFIG[i]?.visualLength) {
        const start = joints[i]
        const end = joints[i + 1]

        if (start && end) {
          const samplePoints = this.sampleSegmentPoints(start, end)

          for (const point of samplePoints) {
            // Check against OBBs first (rotated obstacles)
            for (const obb of obbObstacles) {
              if (this.intersectsSphereOBB(point, segRadius, obb)) {
                return true // HIT!
              }
            }
            // Then check against AABBs
            const sphere = new THREE.Sphere(point, segRadius)
            for (const box of boxObstacles) {
              if (box.intersectsSphere(sphere)) {
                return true // HIT!
              }
            }
          }
        }
      }
    }

    // 5. CHECK JOINT SPHERES vs all obstacles
    for (let index = 0; index < this.CONFIG.length; index++) {
      const cfg = this.CONFIG[index]!
      if (cfg.type === 'revolute' || index === this.CONFIG.length - 1) {
        const jointPos = joints[index]
        if (jointPos) {
          // Use different radius for tip vs regular joints
          const isTip = index === this.CONFIG.length - 1
          const checkRadius = isTip
            ? this.TIP_RADIUS + this.OBSTACLE_COLLISION_MARGIN
            : jointRadius

          // Check against OBBs first
          for (const obb of obbObstacles) {
            if (this.intersectsSphereOBB(jointPos, checkRadius, obb)) {
              return true // HIT!
            }
          }
          // Then check against AABBs
          const sphere = new THREE.Sphere(jointPos, checkRadius)
          for (const box of boxObstacles) {
            if (box.intersectsSphere(sphere)) {
              return true // HIT!
            }
          }
        }
      }
    }

    // No collisions detected - configuration is valid
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

  // ==========================================================================
  // SELF-COLLISION DETECTION
  // ==========================================================================
  //
  // WHY IS THIS NEEDED?
  // --------------------
  // When a robot has multiple joints, it can bend into configurations where
  // its own arm segments intersect each other. This is called "self-collision".
  //
  // For example, imagine a robot arm that bends back on itself - the forearm
  // might pass through the upper arm. The RRT planner needs to detect and
  // reject these invalid configurations.
  //
  // THE ALGORITHM (Analytic Segment Distance):
  // ------------------------------------------
  // We model each link segment as a "capsule" (a cylinder with rounded ends).
  // To check if two capsules intersect, we:
  //   1. Compute the minimum distance between the two segment centerlines
  //      using an analytic formula (closestDistanceBetweenSegments)
  //   2. If distance < 2 * collisionRadius, the capsules overlap
  //
  // JOINT PROXIMITY HANDLING:
  // -------------------------
  // Adjacent segments naturally meet at joints. To avoid false positives near
  // these connection points, we TRIM each segment from both ends before checking.
  // This effectively ignores the "joint region" where segments are expected to
  // be close together.
  //
  // WHICH SEGMENTS TO CHECK:
  // ------------------------
  // We only check NON-ADJACENT segments. Adjacent segments (like the upper arm
  // and forearm) naturally connect at a joint, so they will always "touch" there.
  //
  // If we have segments: [0-1], [1-2], [2-3], [3-4]
  //   - Segment [0-1] checks against [2-3], [3-4] (skips adjacent [1-2])
  //   - Segment [1-2] checks against [3-4] (skips adjacent [2-3])
  //   - And so on...
  //
  // ==========================================================================

  /**
   * Checks if the robot is colliding with itself at a given configuration.
   *
   * This is essential for robots with many joints, where the arm can bend
   * back on itself and cause segments to intersect.
   *
   * Uses analytic segment-to-segment distance calculation for accuracy,
   * with endpoint trimming to handle natural joint proximity.
   *
   * @param angles - The joint angles to check (in radians)
   * @returns true if there is a self-collision, false if configuration is valid
   */
  public checkSelfCollision(angles: number[]): boolean {
    // Step 1: Get the 3D positions of all joints in the robot
    // This gives us an array of Vector3 positions: [Base, Joint1, Joint2, ..., Tip]
    const jointPositions = this.getJointPositions(angles)

    // Step 2: Get self-collision radius (uses smaller margin than obstacle check)
    // Two capsules collide when their centerlines are closer than 2 * radius
    const collisionRadius = this.getSelfCollisionRadius()
    const collisionThreshold = 2 * collisionRadius

    // Step 3: Build a list of segments with their raw endpoints
    // Each segment connects joint[i] to joint[i+1]
    const segments: {
      start: THREE.Vector3
      end: THREE.Vector3
      index: number
    }[] = []

    for (let i = 0; i < jointPositions.length - 1; i++) {
      // Only include segments that have a visual representation
      // (Some joints might just be rotation points without physical links)
      if (this.CONFIG[i]?.visualLength && this.CONFIG[i]!.visualLength! > 0) {
        const start = jointPositions[i]
        const end = jointPositions[i + 1]
        if (start && end) {
          segments.push({ start, end, index: i })
        }
      }
    }

    // Step 4: Check each pair of NON-ADJACENT segments for intersection
    // We iterate through all pairs where j > i + 1 (skipping adjacent segments)
    for (let i = 0; i < segments.length; i++) {
      for (let j = i + 2; j < segments.length; j++) {
        // Why j = i + 2?
        // - j = i would be the same segment (skip)
        // - j = i + 1 is the adjacent segment that shares a joint (skip)
        // - j = i + 2 and beyond are non-adjacent segments we need to check

        const segA = segments[i]!
        const segB = segments[j]!

        // Trim both segments to avoid false positives near joints
        // This shrinks each segment from both ends by SELF_COLLISION_TRIM
        const trimmedA = this.trimSegment(
          segA.start,
          segA.end,
          this.SELF_COLLISION_TRIM
        )
        const trimmedB = this.trimSegment(
          segB.start,
          segB.end,
          this.SELF_COLLISION_TRIM
        )

        // Compute the minimum distance between the two trimmed segments
        const dist = this.closestDistanceBetweenSegments(
          trimmedA.start,
          trimmedA.end,
          trimmedB.start,
          trimmedB.end
        )

        // If distance is less than the collision threshold, segments overlap
        if (dist < collisionThreshold) {
          return true // Self-collision detected!
        }
      }
    }

    // No self-collisions found - configuration is valid
    return false
  }
}
