import * as THREE from 'three'
import { Robot } from './Robot'

interface Node {
  angles: number[]
  parent: Node | null
}

export interface RRTParams {
  stepSize: number
  maxIter: number
  goalBias: number
  algorithm?: 'standard' | 'connect'
}

export class RRTPlanner {
  private robot: Robot
  private obstacle: THREE.Mesh
  private TIME_LIMIT_MS = 3000 // Increased to 3 seconds for complex queries

  // Visualization helper
  public lastTrees: Node[] = []

  constructor(robot: Robot, obstacle: THREE.Mesh) {
    this.robot = robot
    this.obstacle = obstacle
  }

  public plan(
    startAngles: number[],
    targetPos: THREE.Vector3,
    params: RRTParams = {
      stepSize: 0.05,
      maxIter: 20000,
      goalBias: 0.05,
      algorithm: 'connect',
    }
  ): number[][] | null {
    this.lastTrees = [] // Reset visualization

    // --- STEP 1: FIND A VALID GOAL (Constraint Aware Inverse Kinematics) ---
    // this is important for the RRT-connect version which starts exploring path from the target position as well as from the start position of the tip.

    let goalAngles: number[] | null = null

    const solution = this.robot.solveRobustIK(targetPos, this.obstacle)

    // this step checks for collisions based on the angles given by solveRobustIK
    // The planner needs to know: "Did the IK give me a perfect goal, or a broken one?"
    // If perfect: Great, we trust it fully.
    // If broken (colliding): We still use it as a rough guide ("Bias"), but we log a warning.

    if (!this.robot.checkCollision(solution, this.obstacle)) {
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
    if (this.robot.checkCollision(goalAngles, this.obstacle)) {
      console.warn(
        'Goal in collision. Searching for valid neighbor for RRT-Connect...'
      )
      const validNeighbor = this.findValidNeighbor(goalAngles)
      if (validNeighbor) {
        console.log('Found valid neighbor goal.')
        goalAngles = validNeighbor
      } else {
        console.warn(
          'Could not find valid neighbor. RRT-Connect might fail or be invalid.'
        )
        return null
      }
    }

    return this.planConnect(startAngles, goalAngles, params)
  }

  /**
   * Tries to find a collision-free configuration near a given point.
   * Useful when the IK solution is slightly inside a wall.
   */
  private findValidNeighbor(
    angles: number[],
    attempts = 100,
    range = 0.5
  ): number[] | null {
    for (let i = 0; i < attempts; i++) {
      const candidate = angles.map(
        (a) => a + (Math.random() * range * 2 - range)
      )
      if (!this.robot.checkCollision(candidate, this.obstacle)) {
        return candidate
      }
    }
    return null
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
      return !this.robot.checkCollision(to, this.obstacle)
    }

    const steps = Math.ceil(dist / RESOLUTION)

    for (let i = 1; i <= steps; i++) {
      const t = i / steps
      // Linear interpolation between joint angles
      const intermediate = from.map((val, idx) => val + (to[idx]! - val) * t)
      if (this.robot.checkCollision(intermediate, this.obstacle)) {
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
  ): number[][] | null {
    const { stepSize, maxIter, goalBias } = params
    const limits = this.robot.getLimits()
    const tree: Node[] = [{ angles: startAngles, parent: null }]
    this.lastTrees = tree

    const startTime = performance.now()

    for (let i = 0; i < maxIter; i++) {
      if (performance.now() - startTime > this.TIME_LIMIT_MS) {
        console.warn('Standard RRT Timeout')
        this.lastTrees = tree
        // Return null to indicate failure, but tree is preserved in this.lastTrees
        return null
      }

      // 1. Sample
      let sample: number[] = []
      // Use goalAngles for bias (even if it's slightly inside wall, it pulls us in right direction)
      if (Math.random() < goalBias) {
        sample = [...goalAngles]
      } else {
        for (let d = 0; d < limits.length; d++) {
          const min = limits[d]![0]
          const max = limits[d]![1]
          sample.push(Math.random() * (max - min) + min)
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
          return path.reverse()
        }
      }
    }
    return null
  }

  private planConnect(
    startAngles: number[],
    goalAngles: number[],
    params: RRTParams
  ): number[][] | null {
    const { stepSize, maxIter, goalBias } = params
    const limits = this.robot.getLimits()

    // TWO TREES we are going to find a path from start to goal and from goal to start
    const startTree: Node[] = [{ angles: startAngles, parent: null }]
    const goalTree: Node[] = [{ angles: goalAngles, parent: null }]

    let treeA = startTree
    let treeB = goalTree

    const startTime = performance.now()

    for (let i = 0; i < maxIter; i++) {
      // Because this is an algorithm with potentially a very large number of iterations, we need to limit the time it can run for.
      if (performance.now() - startTime > this.TIME_LIMIT_MS) {
        console.warn('RRT-Connect Timeout')
        this.lastTrees = [...startTree, ...goalTree] // Visualize partial trees
        return null
      }

      // 1. Random Sample (Respecting joint angle limits) with Goal Bias
      let sample: number[] = []
      if (goalAngles && Math.random() < goalBias) {
        sample = [...goalAngles]
      } else {
        // Standard Random Sample
        for (let d = 0; d < limits.length; d++) {
          const min = limits[d]![0]
          const max = limits[d]![1]
          sample.push(Math.random() * (max - min) + min)
        }
      }

      // 2. Extend Tree A towards Sample
      const newNodeA = this.extend(treeA, sample, stepSize)

      if (newNodeA) {
        // 3. IF Tree A successfully grew, try to connect Tree B directly to that new node
        const newNodeB = this.connect(treeB, newNodeA.angles, stepSize)

        if (newNodeB) {
          // SUCCESS! The trees met.
          console.log(`Connected after ${i} iterations!`)

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

          return finalPath
        }
      }

      // 4. Swap Trees (Grow from the other side next time)
      const temp = treeA
      treeA = treeB
      treeB = temp
    }

    // Visual debugging (Show both trees)
    this.lastTrees = [...startTree, ...goalTree]
    return null
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

      // If very close, we connected!
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
