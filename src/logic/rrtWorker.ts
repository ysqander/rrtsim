import * as THREE from 'three'
import { Robot, type LinkConfig } from './Robot'
import { RRTPlanner, type RRTParams, type FailureReason } from './RRTPlanner'

// Input data sent from main thread to worker
export interface WorkerInput {
  startAngles: number[]
  targetPos: { x: number; y: number; z: number }
  params: RRTParams
  robotConfig: LinkConfig[]
  obstacles: { min: [number, number, number]; max: [number, number, number] }[]
}

// Output data sent from worker back to main thread
export interface WorkerOutput {
  path: number[][] | null
  treeData: { angles: number[]; parentIndex: number | null }[]
  success: boolean
  time: number
  failureReason?: FailureReason // Why planning failed (if it did)
  failureDetails?: string // Human-readable failure explanation
  startNodes?: number // Number of nodes in the start tree (RRT-Connect)
  goalNodes?: number // Number of nodes in the goal tree (RRT-Connect)
  meetIteration?: number // Iteration at which the trees met (RRT-Connect)
  error?: string // Optional error message if planning failed due to an exception
}

// Worker message handler
self.onmessage = (e: MessageEvent<WorkerInput>) => {
  const { startAngles, targetPos, params, robotConfig, obstacles } = e.data
  const start = performance.now()

  try {
    // Reconstruct Three.js objects from serialized data
    const robot = new Robot(robotConfig)
    const boxes = obstacles.map(
      (b) =>
        new THREE.Box3(new THREE.Vector3(...b.min), new THREE.Vector3(...b.max))
    )
    const planner = new RRTPlanner(robot, boxes)

    // Run planning - now returns PlanResult with failure info
    const target = new THREE.Vector3(targetPos.x, targetPos.y, targetPos.z)
    const result = planner.plan(startAngles, target, params)

    // Serialize tree for visualization (strip circular parent refs)
    // We convert the Node[] with parent references to a flat array with parentIndex
    const treeData = planner.lastTrees.map((node) => ({
      angles: node.angles,
      parentIndex: node.parent ? planner.lastTrees.indexOf(node.parent) : null,
    }))

    // Send result back to main thread with failure reason if applicable
    self.postMessage({
      path: result.path,
      treeData,
      success: !!result.path,
      time: Math.round(performance.now() - start),
      failureReason: result.failureReason,
      failureDetails: result.failureDetails,
      startNodes: result.startNodes,
      goalNodes: result.goalNodes,
      meetIteration: result.meetIteration,
    } as WorkerOutput)
  } catch (error) {
    // If anything goes wrong, send back a failure response
    // This prevents the worker from silently crashing and leaving isWorkerPlanning = true
    console.error('[RRT Worker] Error during planning:', error)
    self.postMessage({
      path: null,
      treeData: [],
      success: false,
      time: Math.round(performance.now() - start),
      error: error instanceof Error ? error.message : 'Unknown error',
    } as WorkerOutput)
  }
}
