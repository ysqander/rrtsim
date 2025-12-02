- What do the Hitboxes show at the moment. seems like gibberish.
- Run the calcs in "worker" to avoid crashing when people stress test it.

Improvements for learning outcomes and UX
Bold gating per step
Disable irrelevant controls per step and add micro-copy with “why”: e.g., in Step 1 lock RRT controls; in Step 2/3 lock Greedy.
Provide step intros/outros with “What you should notice” and “Try this next”.

Deterministic runs for fair comparisons
Add a seeded RNG for RRT in both main thread and worker; expose seed controls: Run, Run (same seed), New seed.
Use the same seed to compare Standard vs Connect so users see true algorithmic differences, not luck.

Clear “why failed” feedback
When planning fails or Greedy collides, show a reason: unreachable target, obstacle collision, self-collision, or iteration budget exhausted.
Surface suggestions: increase max iterations, tweak goal bias/step size, move target.
Parameter guard rails and presets

Provide presets (Beginner, Balanced, Fast, Thorough) and clamp ranges to avoid pathological settings (e.g., extremely small step size or wild goalBias).
Show guidance text under sliders explaining tradeoffs.

Progress, cancel, and replay
Send progress updates from the worker (nodes explored so far, best distance to goal) to animate a progress bar and enable a Cancel button that cleanly aborts the worker plan.

Add “Replay animation” and “Show path smoothing” toggles.
Visualization upgrades that teach
Color the two RRT-Connect trees differently and overlay with a legend; optionally animate both expansions in different hues.
Show a small angle timeline when executing a path (chart per joint) to connect C-space motion to workspace motion.
Offer optional “path smoothing” visualization (short-cutting) to contrast raw vs smoothed path length and execution time.

Scaffolded exercises
End each step with a micro-challenge: “Make Greedy fail”, “Tune goalBias to reduce nodes by 30%”, “Find a case where Standard fails but Connect succeeds.”
Add a 2–3 question mini-quiz per step to check understanding.

Robustness and stress-test guardrails
Unreachable target detection up-front
Quick geometry check before planning (e.g., target beyond total link length or inside a minimum reach envelope) with a clear message and suggested actions.
Worker lifecycle and cancellation
Cancel any in-flight plan on drag or step change; send a cancel message and terminate/recreate the worker if needed to avoid dangling “planning” states.
Progress + time budget
Provide a time/iteration budget (e.g., 1–3 seconds or N nodes). If exceeded, stop and report partial insights (closest distance to goal).
Tree visualization performance
Downsample tree rendering for large searches (render every k-th edge, or cap the number of drawn edges).
Build treeData without indexOf per node to avoid O(n^2) during serialization; attach a stable id or parent index during growth.
Angle wrap and distance metric
Consider shortest-angle wrap-around in distance/steer to avoid unnecessarily long rotations across the -π/π boundary and to improve steering quality.
Collision tolerances explained and adjustable
Offer advanced toggles to visualize/adjust margins for obstacle vs self-collision so learners see why a configuration is invalid and how safety buffers affect planning.
Path validation and smoothing post-check
Re-validate the found path at a finer resolution (and after smoothing) to avoid rare tunneling issues and give users confidence in the final trajectory.
Small UI/teaching polish
Add step-specific callouts in the panel: “What’s new in this step” and “What to notice.”
Provide a “Reset scenario” and “Restore defaults” button.
Keep the “controls help” visible or collapsible; add a “?” tooltip to RRT parameters.
Optional “compare runs” card that stores last run’s metrics for side-by-side with the current run.
Brief status: I reviewed the planner, collision logic, worker communication, and the stepper UI. The recommendations above focus on making outcomes deterministic and explainable, while adding guard rails and richer visuals that teach core concepts clearly.
Add seed control and deterministic comparisons.
Add progress/cancel and failure reasons.
Guard rails (presets, clamped ranges, time/iteration budgets).
Visualization upgrades (dual-tree colors, joint angle charts, path smoothing).
Robustness (unreachable detection, worker cancellation, efficient tree serialization).
Scaffolded exercises and brief quizzes per step.
