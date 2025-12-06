import * as THREE from 'three'
import { Robot } from './Robot'

// Seeded PRNG (mulberry32) - deterministic random when seed provided
function mulberry32(seed: number): () => number {
  return () => {
    let t = (seed += 0x6d2b79f5)
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }
}

interface Node {
  angles: number[]
  parent: Node | null
}

export interface RRTParams {
  stepSize: number
  maxIter: number
  goalBias: number
  algorithm?: 'standard' | 'connect'
  seed?: number // Optional: makes sampling deterministic for reproducible results
}

// Failure reason types for detailed feedback
export type FailureReason =
  | 'timeout' // Iteration/time budget exhausted
  | 'unreachable' // Target not reachable (IK fails completely)
  | 'goal_in_collision' // Goal configuration collides with obstacle
  | 'self_collision' // Robot would collide with itself
  | null // Success (no failure)

export interface PlanResult {
  path: number[][] | null
  failureReason: FailureReason
  failureDetails?: string // Optional human-readable details
  startNodes?: number // Number of nodes in the start tree (RRT-Connect)
  goalNodes?: number // Number of nodes in the goal tree (RRT-Connect)
  meetIteration?: number // Iteration at which the trees met (RRT-Connect)
}

export class RRTPlanner {
  private robot: Robot
  // Supports both Box3 (AABB) and OBB obstacles for accurate rotated collision
  private obstacleBoxes: (
    | THREE.Box3
    | { halfSize: [number, number, number]; matrix: number[] }
  )[]
  private TIME_LIMIT_MS = 3000 // Increased to 3 seconds for complex queries
  private random: () => number = Math.random // Default to Math.random, seeded in plan()

  // Visualization helper
  public lastTrees: Node[] = []

  // Accept Mesh, Box3, OBB, or arrays of any combination
  constructor(
    robot: Robot,
    obstacle:
      | THREE.Mesh
      | THREE.Box3
      | { halfSize: [number, number, number]; matrix: number[] }
      | Array<
          | THREE.Mesh
          | THREE.Box3
          | { halfSize: [number, number, number]; matrix: number[] }
        >
  ) {
    this.robot = robot
    const arr = Array.isArray(obstacle) ? obstacle : [obstacle]
    // Normalize: keep Box3s and OBBs as-is; convert Mesh -> Box3
    this.obstacleBoxes = arr.map((o) => {
      // Check for Box3
      if (
        o &&
        typeof o === 'object' &&
        'isBox3' in o &&
        (o as THREE.Box3).isBox3
      ) {
        return o as THREE.Box3
      }
      // Check for OBB
      if (o && typeof o === 'object' && 'halfSize' in o && 'matrix' in o) {
        return o as { halfSize: [number, number, number]; matrix: number[] }
      }
      // Fallback: Mesh -> Box3
      return new THREE.Box3().setFromObject(o as THREE.Mesh)
    })
  }

  public plan(
    startAngles: number[],
    targetPos: THREE.Vector3,
    params: RRTParams = {
      stepSize: 0.05,
      maxIter: 20000,
      goalBias: 0.0,
      algorithm: 'connect',
    }
  ): PlanResult {
    // Initialize random function: seeded if seed provided, otherwise Math.random
    this.random =
      params.seed !== undefined ? mulberry32(params.seed) : Math.random

    this.lastTrees = [] // Reset visualization

    // --- STEP 0: CHECK IF TARGET IS EVEN REACHABLE ---
    // Quick check: is the target within the robot's maximum reach?
    const maxReach = this.robot.CONFIG.reduce((sum, cfg) => {
      return sum + (cfg.visualLength || 0)
    }, 0)
    const targetDist = targetPos.length() // Distance from origin (robot base)
    if (targetDist > maxReach * 0.95) {
      // 95% of max reach as safety margin
      console.warn('Target appears to be beyond robot reach')
      return {
        path: null,
        failureReason: 'unreachable',
        failureDetails: `Target distance (${targetDist.toFixed(
          2
        )}) exceeds robot reach (${maxReach.toFixed(2)})`,
      }
    }

    // --- STEP 1: FIND A VALID GOAL (Constraint Aware Inverse Kinematics) ---
    // This is important for the RRT-Connect version which starts exploring a path from the target position as well as from the start position of the robot.

    let goalAngles: number[] | null = null

    const solution = this.robot.solveRobustIK(
      targetPos,
      this.obstacleBoxes,
      this.random
    )

    // this step checks for collisions based on the angles given by solveRobustIK
    // The planner needs to know: "Did the IK give me a perfect goal, or a broken one?"
    // If perfect: Great, we trust it fully.
    // If broken (colliding): We still use it as a rough guide ("Bias"), but we log a warning.

    // Check for self-collision first
    if (this.robot.checkSelfCollision(solution)) {
      console.warn('IK solution results in self-collision')
      // Still try to plan, but note this for potential failure
    }

    if (!this.robot.checkCollision(solution, this.obstacleBoxes)) {
      goalAngles = solution
      console.log('Found valid goal config.')
    } else {
      // Even robust IK failed
      console.warn(
        'IK failed to find collision-free goal. Using best-effort for bias.'
      )
      // FALLBACK: Use the colliding solution for BIAS, but don't trust it 100%
      // We will change the success condition to check Task Space distance.
      goalAngles = solution
    }

    // Dispatch based on RRT algorithm version standard or connect
    if (params.algorithm === 'standard') {
      return this.planStandard(startAngles, goalAngles, targetPos, params)
    }

    // --- STEP 2: RRT-CONNECT (Default) ---

    // CRITICAL: RRT-Connect REQUIRES a valid goal configuration.
    // If the exact IK solution is in collision, try to find a valid configuration nearby.
    if (this.robot.checkCollision(goalAngles, this.obstacleBoxes)) {
      console.warn(
        'Goal in collision. Searching for valid neighbor for RRT-Connect...'
      )
      const validNeighbor = this.findValidNeighbor(goalAngles, targetPos)
      if (validNeighbor) {
        console.log('Found valid neighbor goal.')
        goalAngles = validNeighbor
      } else {
        console.warn(
          'Could not find valid neighbor. RRT-Connect might fail or be invalid.'
        )
        return {
          path: null,
          failureReason: 'goal_in_collision',
          failureDetails:
            'Target position requires a configuration that collides with obstacles. Try moving the target.',
        }
      }
    }

    return this.planConnect(startAngles, goalAngles, params, targetPos)
  }

  /**
   * Tries to find a collision-free configuration near a given point
   * that ALSO reaches the target position.
   * Uses progressive thresholds and returns the best collision-free candidate
   * even if strict thresholds aren't met.
   * Useful when the IK solution is slightly inside a wall.
   */
  private findValidNeighbor(
    angles: number[],
    targetPos: THREE.Vector3,
    attempts = 300,
    range = 0.8
  ): number[] | null {
    let best: number[] | null = null
    let bestDist = Infinity

    // Progressive thresholds - return early if we find something good enough
    const thresholds = [0.2, 0.3, 0.4, 0.5]
    const perTier = Math.max(1, Math.floor(attempts / thresholds.length))

    for (const thr of thresholds) {
      for (let i = 0; i < perTier; i++) {
        const candidate = angles.map(
          (a) => a + (this.random() * range * 2 - range)
        )
        if (!this.robot.checkCollision(candidate, this.obstacleBoxes)) {
          const tip = this.robot.getTipPosition(candidate)
          const d = tip.distanceTo(targetPos)
          if (d < bestDist) {
            best = candidate
            bestDist = d
          }
          // Return early if within current threshold
          if (d < thr) return candidate
        }
      }
    }

    // Return best collision-free neighbor found (may help trees connect)
    return best
  }

  /**
   * Checks if the path segment between 'from' and 'to' is collision-free
   * by interpolating and checking intermediate configurations.
   *
   * This prevents "tunneling" where the robot might jump over an obstacle
   * if the step size is too large.
   */
  private isSegmentValid(from: number[], to: number[]): boolean {
    const dist = this.distance(from, to)
    const RESOLUTION = 0.05 // Check every 0.05 radians (approx 3 degrees)

    // If distance is very small, just check the endpoint (optimization)
    if (dist < RESOLUTION) {
      return !this.robot.checkCollision(to, this.obstacleBoxes)
    }

    const steps = Math.ceil(dist / RESOLUTION)

    for (let i = 1; i <= steps; i++) {
      const t = i / steps
      // Linear interpolation between joint angles
      const intermediate = from.map((val, idx) => val + (to[idx]! - val) * t)
      if (this.robot.checkCollision(intermediate, this.obstacleBoxes)) {
        return false // Hit obstacle during transition
      }
    }
    return true
  }

  private planStandard(
    startAngles: number[],
    goalAngles: number[],
    targetPos: THREE.Vector3,
    params: RRTParams
  ): PlanResult {
    const { stepSize, maxIter, goalBias } = params
    const limits = this.robot.getLimits()
    const tree: Node[] = [{ angles: startAngles, parent: null }]
    this.lastTrees = tree

    const startTime = performance.now()
    let timedOut = false

    for (let i = 0; i < maxIter; i++) {
      if (performance.now() - startTime > this.TIME_LIMIT_MS) {
        console.warn('Standard RRT Timeout')
        this.lastTrees = tree
        timedOut = true
        break
      }

      // 1. Sample
      let sample: number[] = []
      // Use goalAngles for bias (even if it's slightly inside wall, it pulls us in right direction)
      if (this.random() < goalBias) {
        sample = [...goalAngles]
      } else {
        for (let d = 0; d < limits.length; d++) {
          const min = limits[d]![0]
          const max = limits[d]![1]
          sample.push(this.random() * (max - min) + min)
        }
      }

      // 2. Extend
      const newNode = this.extend(tree, sample, stepSize)

      if (newNode) {
        // 3. Check if reached goal
        // ROBUST CHECK: Compare End Effector Position vs Target Position
        // This handles cases where goalAngles might be slightly off or invalid
        const tipPos = this.robot.getTipPosition(newNode.angles)
        if (tipPos.distanceTo(targetPos) < 0.2) {
          console.log(`Standard RRT Success after ${i} nodes`)
          const path = this.getPathFromRoot(newNode)
          return {
            path: path.reverse(),
            failureReason: null,
          }
        }
      }
    }

    // Planning failed - determine reason
    this.lastTrees = tree

    if (timedOut) {
      return {
        path: null,
        failureReason: 'timeout',
        failureDetails: `Time limit (${this.TIME_LIMIT_MS}ms) exceeded. Try increasing step size or max iterations.`,
      }
    }

    // Max iterations reached without timeout
    return {
      path: null,
      failureReason: 'timeout',
      failureDetails: `Max iterations (${maxIter}) reached. Try increasing iterations or adjusting parameters.`,
    }
  }

  private planConnect(
    startAngles: number[],
    goalAngles: number[],
    params: RRTParams,
    targetPos: THREE.Vector3
  ): PlanResult {
    const { stepSize, maxIter, goalBias } = params
    const limits = this.robot.getLimits()

    // TWO TREES we are going to find a path from start to goal and from goal to start
    const startTree: Node[] = [{ angles: startAngles, parent: null }]
    const goalTree: Node[] = [{ angles: goalAngles, parent: null }]

    let treeA = startTree
    let treeB = goalTree

    const startTime = performance.now()
    let timedOut = false

    for (let i = 0; i < maxIter; i++) {
      // Because this is an algorithm with potentially a very large number of iterations, we need to limit the time it can run for.
      if (performance.now() - startTime > this.TIME_LIMIT_MS) {
        console.warn('RRT-Connect Timeout')
        this.lastTrees = [...startTree, ...goalTree] // Visualize partial trees
        timedOut = true
        break
      }

      // 1. We take a random point to extend the tree towards just like in standard RTT see planStandard function
      let sample: number[] = []
      if (goalAngles && this.random() < goalBias) {
        sample = [...goalAngles]
      } else {
        // Standard Random Sample
        for (let d = 0; d < limits.length; d++) {
          const min = limits[d]![0]
          const max = limits[d]![1]
          sample.push(this.random() * (max - min) + min)
        }
      }

      // 2. Extend Tree A towards Sample
      const newNodeA = this.extend(treeA, sample, stepSize)

      if (newNodeA) {
        // 3. IF Tree A successfully grew, try to connect Tree B directly to that new node
        // NOTE: We connect in JOINT SPACE only - targetPos is NOT used here.
        // The trees should meet wherever they can, not necessarily at the goal.
        const newNodeB = this.connect(treeB, newNodeA.angles, stepSize)

        if (newNodeB) {
          // SUCCESS! The trees met.
          const meetIteration = i
          console.log(`Connected after ${meetIteration} iterations!`)

          // Updated the last trees to show the paths
          this.lastTrees = [...startTree, ...goalTree]

          // Reconstruct path: Start -> ... -> MeetPoint -> ... -> Goal
          // Note: We have to handle which tree is which (since we swap them)
          const pathA = this.getPathFromRoot(newNodeA)
          const pathB = this.getPathFromRoot(newNodeB)

          // Force the exact goal configuration to be the last point
          // This ensures visual "snapping" to the target
          const finalPath: number[][] = []

          // If treeA is startTree, pathA is Start->Middle. pathB is Goal->Middle
          if (treeA === startTree) {
            finalPath.push(...pathA.reverse(), ...pathB)
          } else {
            finalPath.push(...pathB.reverse(), ...pathA)
          }

          // Append the goalAngles explicitly if not already there
          // pathB started at Goal, so it's likely already there, but let's be safe
          if (goalAngles) finalPath.push(goalAngles)

          // --- Final snap-to-target: try to connect to a valid IK goal
          //     so the tip actually hits the specified target.
          try {
            const desired = this.robot.solveRobustIK(
              targetPos,
              this.obstacleBoxes,
              this.random
            )
            const last = finalPath[finalPath.length - 1]!
            const snapTree: Node[] = [{ angles: last, parent: null }]
            // Slightly smaller step helps near obstacles
            const snapNode = this.connect(
              snapTree,
              desired,
              Math.max(0.01, stepSize * 0.75)
            )
            if (snapNode) {
              const snapPath = this.getPathFromRoot(snapNode)
              // Drop the first (duplicate) node
              finalPath.push(...snapPath.slice(1))
            }
          } catch {
            console.warn('Snap-to-target failed, using meet configuration.')
          }

          return {
            path: finalPath,
            failureReason: null,
            startNodes: startTree.length,
            goalNodes: goalTree.length,
            meetIteration,
          }
        }
      }

      // 4. Swap Trees (Grow from the other side next time)
      const temp = treeA
      treeA = treeB
      treeB = temp
    }

    // Visual debugging (Show both trees)
    this.lastTrees = [...startTree, ...goalTree]

    // Planning failed - determine reason
    // IMPORTANT: Always return startNodes/goalNodes so both trees can be visualized
    if (timedOut) {
      return {
        path: null,
        failureReason: 'timeout',
        failureDetails: `Time limit (${this.TIME_LIMIT_MS}ms) exceeded. Try increasing step size or max iterations.`,
        startNodes: startTree.length,
        goalNodes: goalTree.length,
      }
    }

    // Max iterations reached without timeout
    return {
      path: null,
      failureReason: 'timeout',
      failureDetails: `Max iterations (${maxIter}) reached. Try increasing iterations or adjusting parameters.`,
      startNodes: startTree.length,
      goalNodes: goalTree.length,
    }
  }

  // Tries to extend the tree towards a point.
  // Returns the new Node if valid, null if blocked.
  private extend(
    tree: Node[],
    targetAngles: number[],
    stepSize: number
  ): Node | null {
    // Find nearest
    let nearest = tree[0]
    let minDist = Infinity
    for (const node of tree) {
      const d = this.distance(node.angles, targetAngles)
      if (d < minDist) {
        minDist = d
        nearest = node
      }
    }

    // Steer
    const newAngles = this.steer(nearest!.angles, targetAngles, stepSize)

    // Avoid duplicating nodes when steer doesn't move (e.g., sampling the root/goal)
    if (this.distance(nearest!.angles, newAngles) < 1e-6) {
      return null
    }

    // Check Collision (Segment Check)
    if (this.isSegmentValid(nearest!.angles, newAngles)) {
      const newNode: Node = { angles: newAngles, parent: nearest! }
      tree.push(newNode)
      return newNode
    }
    return null
  }

  // Tries to aggressively connect the tree to a target point until it hits a wall
  // Returns the final node reached (if it reached the target), or null
  // NOTE: This uses JOINT SPACE only - no task-space validation.
  // The trees meet wherever they can in configuration space, not necessarily at the goal position.
  private connect(
    tree: Node[],
    targetAngles: number[],
    stepSize: number
  ): Node | null {
    let nearest = tree[0]
    let minDist = Infinity

    // Find nearest node in THIS tree to the target
    for (const node of tree) {
      const d = this.distance(node.angles, targetAngles)
      if (d < minDist) {
        minDist = d
        nearest = node
      }
    }

    let currentNode = nearest!

    // Keep stepping until we hit the target or a wall
    while (true) {
      const newAngles = this.steer(currentNode.angles, targetAngles, stepSize)

      // Collision Check (Segment Check)
      if (!this.isSegmentValid(currentNode.angles, newAngles)) {
        return null // Hit wall
      }

      // Add to tree
      const newNode: Node = { angles: newAngles, parent: currentNode }
      tree.push(newNode)
      currentNode = newNode

      // Success: close enough in joint space to be considered connected
      if (this.distance(newAngles, targetAngles) < 0.1) {
        return newNode
      }

      // If steer didn't move us (we are stuck or at target), break
      if (this.distance(newAngles, currentNode.parent!.angles) < 0.001) {
        return null
      }
    }
  }

  private getPathFromRoot(node: Node): number[][] {
    const path: number[][] = []
    let curr: Node | null = node
    while (curr) {
      path.push(curr.angles)
      curr = curr.parent
    }
    // Returns [Tip, ..., Root]
    return path
  }

  private distance(a: number[], b: number[]): number {
    let sum = 0
    for (let i = 0; i < a.length; i++) sum += Math.pow(a[i]! - b[i]!, 2)
    return Math.sqrt(sum)
  }

  private steer(from: number[], to: number[], stepSize: number): number[] {
    const dist = this.distance(from, to)
    if (dist < stepSize) return to
    const ratio = stepSize / dist
    return from.map((val, i) => val + (to[i]! - val) * ratio)
  }
}
