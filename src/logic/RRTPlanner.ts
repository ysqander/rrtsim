import * as THREE from 'three'
import { Robot } from './Robot'

interface Node {
  angles: number[]
  parent: Node | null
}

export class RRTPlanner {
  private robot: Robot
  private obstacle: THREE.Mesh
  private stepSize = 0.2
  private TIME_LIMIT_MS = 1000 // Give it a full second
  private maxIter = 10000

  // Visualization helper
  public lastTrees: Node[] = []

  constructor(robot: Robot, obstacle: THREE.Mesh) {
    this.robot = robot
    this.obstacle = obstacle
  }

  public plan(
    startAngles: number[],
    targetPos: THREE.Vector3
  ): number[][] | null {
    this.lastTrees = [] // Reset visualization
    const limits = this.robot.getLimits()

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
        'Target is unreachable (Inside Wall or Kinematically impossible).'
      )
      this.lastTrees = [{ angles: startAngles, parent: null }]
      return null
    }

    // --- STEP 2: RRT-CONNECT ---
    // TWO TREES we are going to find a path from start to goal and from goal to start
    const startTree: Node[] = [{ angles: startAngles, parent: null }]
    const goalTree: Node[] = [{ angles: goalAngles, parent: null }]

    let treeA = startTree
    let treeB = goalTree

    const startTime = performance.now()

    for (let i = 0; i < this.maxIter; i++) {
      // Because this is an algorithm with potentially a very large number of iterations, we need to limit the time it can run for.
      if (performance.now() - startTime > this.TIME_LIMIT_MS) {
        console.warn('RRT-Connect Timeout')
        break
      }

      // 1. Random Sample (Respecting joint angle limits)
      const sample: number[] = []
      for (let d = 0; d < limits.length; d++) {
        const min = limits[d]![0]
        const max = limits[d]![1]
        sample.push(Math.random() * (max - min) + min)
      }

      // 2. Extend Tree A towards Sample
      const newNodeA = this.extend(treeA, sample)

      if (newNodeA) {
        // 3. IF Tree A successfully grew, try to connect Tree B directly to that new node
        const newNodeB = this.connect(treeB, newNodeA.angles)

        if (newNodeB) {
          // SUCCESS! The trees met.
          console.log(`Connected after ${i} iterations!`)

          // Updated the last trees to show the paths
          this.lastTrees = [...startTree, ...goalTree]

          // Reconstruct path: Start -> ... -> MeetPoint -> ... -> Goal
          // Note: We have to handle which tree is which (since we swap them)
          const pathA = this.getPathFromRoot(newNodeA)
          const pathB = this.getPathFromRoot(newNodeB)

          // If treeA is startTree, pathA is Start->Middle. pathB is Goal->Middle
          if (treeA === startTree) {
            return [...pathA.reverse(), ...pathB]
          } else {
            return [...pathB.reverse(), ...pathA]
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
    return null
  }

  // Tries to extend the tree towards a point.
  // Returns the new Node if valid, null if blocked.
  private extend(tree: Node[], targetAngles: number[]): Node | null {
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
    const newAngles = this.steer(nearest!.angles, targetAngles)

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
  private connect(tree: Node[], targetAngles: number[]): Node | null {
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
      const newAngles = this.steer(currentNode.angles, targetAngles)

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

  private steer(from: number[], to: number[]): number[] {
    const dist = this.distance(from, to)
    if (dist < this.stepSize) return to
    const ratio = this.stepSize / dist
    return from.map((val, i) => val + (to[i]! - val) * ratio)
  }
}
