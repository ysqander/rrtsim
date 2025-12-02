import * as THREE from 'three'
import { Robot, type LinkConfig } from './Robot'
import { RRTPlanner, type RRTParams } from './RRTPlanner'

// Input data sent from main thread to worker
export interface WorkerInput {
  startAngles: number[]
  targetPos: { x: number; y: number; z: number }
  params: RRTParams
  robotConfig: LinkConfig[]
  obstacleBox: { min: [number, number, number]; max: [number, number, number] }
}

// Output data sent from worker back to main thread
export interface WorkerOutput {
  path: number[][] | null
  treeData: { angles: number[]; parentIndex: number | null }[]
  success: boolean
  time: number
}

// Worker message handler
self.onmessage = (e: MessageEvent<WorkerInput>) => {
  const { startAngles, targetPos, params, robotConfig, obstacleBox } = e.data
  const start = performance.now()

  // Reconstruct Three.js objects from serialized data
  const robot = new Robot(robotConfig)
  const box = new THREE.Box3(
    new THREE.Vector3(...obstacleBox.min),
    new THREE.Vector3(...obstacleBox.max)
  )
  const planner = new RRTPlanner(robot, box)

  // Run planning
  const target = new THREE.Vector3(targetPos.x, targetPos.y, targetPos.z)
  const path = planner.plan(startAngles, target, params)

  // Serialize tree for visualization (strip circular parent refs)
  // We convert the Node[] with parent references to a flat array with parentIndex
  const treeData = planner.lastTrees.map((node) => ({
    angles: node.angles,
    parentIndex: node.parent ? planner.lastTrees.indexOf(node.parent) : null,
  }))

  // Send result back to main thread
  self.postMessage({
    path,
    treeData,
    success: !!path,
    time: Math.round(performance.now() - start),
  } as WorkerOutput)
}

