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

    // --- STEP 1: FIND A VALID GOAL (Constraint Aware IK) ---
    // We try multiple times to solve IK. If the solution hits the wall, we randomize and try again.
    let goalAngles: number[] | null = null

    // REFACTORED: Use the robot's built-in Robust IK
    const solution = this.robot.solveRobustIK(targetPos, this.obstacle)

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

    // Dispatch based on algorithm
    if (params.algorithm === 'standard') {
      return this.planStandard(startAngles, goalAngles, targetPos, params)
    }

    // --- STEP 2: RRT-CONNECT (Default) ---
    return this.planConnect(startAngles, goalAngles, params)
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
    this.lastTrees = tree // Visualize this tree

    const startTime = performance.now()

    for (let i = 0; i < maxIter; i++) {
      if (performance.now() - startTime > this.TIME_LIMIT_MS) {
        console.warn('Standard RRT Timeout')
        break
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
          // 20cm tolerance
          console.log(`Standard RRT Success after ${i} nodes`)
          const path = this.getPathFromRoot(newNode)
          // path.unshift(goalAngles) // Don't force specific goal angles if we just reached the position
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
        break
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

    // Check Collision
    if (!this.robot.checkCollision(newAngles, this.obstacle)) {
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

      // Collision Check
      if (this.robot.checkCollision(newAngles, this.obstacle)) {
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
