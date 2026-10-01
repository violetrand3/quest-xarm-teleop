# Commissioning checklist: unresolved requirements before the first supervised movement

Date: 2026-10-01. **Nothing on this list is verified.** Source inspection and a fake backend cannot verify any of it. Each item needs a check on the rig, with explicit approval and nobody near the arm, as for every motion so far.

The adapter's arm command path (`XArmPreparedBackend.command_target`) and gripper command (`GripctlPreparedGripper.command`) raise `LiveIntegrationBlocked` until these are resolved.

## 1. TCP, frame and axis directions
- [ ] Read the TCP offset the controller actually holds. The code never sets it (`set_tcp_offset` is absent from `robotdash`).
- [ ] Confirm `get_position()` reports mm and degrees on this connection (`is_radian=False`, `stepper.py:78`).
- [ ] Confirm the base-frame directions in `stepper.py:137-144` (+x away from base, +y toward the left bin, +z up) on the real arm.
- [ ] Choose and verify `R` (controller LOCAL → base frame) and its signs, one axis at a time, with small moves. The dry-run `R` is an arbitrary example.
- [ ] Decide how LOCAL is kept fixed relative to the room. A recenter or a new app session changes it; the adapter only latches a fault.
- [ ] Confirm that passing back the reported `roll, pitch, yaw` unchanged holds the tool orientation (no drift, no wrist flip near ±180°; `START` has roll 179.86).
- [ ] Decide the scale. 0.2 was a preview value.

## 2. Fresh robot feedback and exclusive command ownership
- [ ] Measure how long `get_position()` and `get_err_warn_code()` take and how fresh the values are. There is no robot-side timestamp.
- [ ] Decide what "fresh" means for feedback read over the SDK connection, and set `feedback_fresh_ms` from measurement.
- [ ] Ensure only one program commands the arm. `stepper.py` holds a long-lived connection and its own STOP; decide which process owns the arm during teleop and how the other is kept out.
- [ ] Decide who performs `motion_enable(True); set_mode(0); set_state(0)`. The adapter never does. It must be an explicit, supervised operator step.
- [ ] Decide the reaction to a non-zero error or warning code. The adapter refuses to continue and never clears errors.

## 3. Stop, cancel and command-queue behaviour
- [ ] Establish whether `set_position(..., wait=False)` queues targets or replaces the current one. The existing code only uses blocking `wait=True` calls in mode 0.
- [ ] If it queues, choose a different interface or a step-wise scheme. The adapter assumes one replaceable target and no backlog.
- [ ] Establish what `emergency_stop()` followed by `set_state(4)` does to a move in progress: stopping distance, time, and resulting state. It is an SDK call, not a hardware E-stop.
- [ ] Establish what is needed to move again after that stop, and confirm nothing resumes by itself.
- [ ] Confirm that not sending a new target does not leave the arm travelling to the last one.
- [ ] Check whether the `speed` and `mvacc` arguments are honoured in the chosen mode, or set limits another way.
- [ ] Measure the delay from a stop request on `linux2` to the arm stopping.

## 4. Approved workspace and motion limits
- [ ] Confirm reduced mode is active on the controller and read back its TCP boundary. `reconfigure.py:55` writes x-min 160; the software fence and your stated limit use 180.
- [ ] Approve the teleop workspace inside the fence (x 180–502, y −300…270, z ≥ 270 mm) and decide the margin.
- [ ] Approve velocity and acceleration limits, the per-engagement displacement limit, the controller-jump limits and the target-to-feedback error limit. The dry-run values are synthetic.
- [ ] Decide the behaviour at the workspace edge. The adapter disengages and latches; it does not clamp.
- [ ] Confirm collision sensitivity (5 in `reconfigure.py:25`) is what is wanted.
- [ ] Decide freshness and stall limits from measured link timing (median RTT about 235 ms; input gaps up to 179 ms seen).
- [ ] Decide whether occasional missing source records (one in a 75 s run) are acceptable as stops, or must be fixed first.

## 5. Physical emergency stop and supervisor
- [ ] A physical emergency stop within reach of a person on site for the whole session.
- [ ] An on-site supervisor who can see the arm and has authority to stop.
- [ ] Nobody inside the arm's reach.
- [ ] An agreed spoken or signalled stop between the remote operator and the supervisor.

## 6. Remote operator's view
- [ ] Live robot-camera feedback that the remote operator can actually use while the Quest input app stays focused. Opening another app or the system menu takes focus away and disengages.
- [ ] Measure the camera latency end to end. The dashboard frames carry no timestamp.
- [ ] Decide what happens when the view freezes.

## 7. Custom gripper
- [ ] Confirm the gripctl calibration is valid and matches 3033 (open) and 1181 (closed) ticks.
- [ ] Decide who enables torque. `POST /api/do/grip` enables it as a side effect.
- [ ] Characterise holding: `grip` has no stall cancel, so it keeps pushing at the effort cap. Decide the effort caps for teleop.
- [ ] Decide the behaviour on input loss and arm disengagement. The adapter sends nothing. gripctl `stop` rewrites the goal to the present position, which may release an object.
- [ ] Handle the 1–3 s USB dropouts: what the adapter does when `status.connected` is false or `age` is large.
- [ ] Verify trigger direction and range on the real jaws, and the pickup tolerance, before enabling following.
- [ ] Decide whether a force limit applies. Reported force is an estimate.

## 8. Input side
- [ ] Decide whether one operator acknowledgement per app start is acceptable (it follows from the unidentified space-change events).
- [ ] Resolve or accept the open `poseInPreviousSpace` direction question. The adapter does not use it.
- [ ] Decide whether wrist-twist coupling into translation (47 mm of controller origin travel in the dry run) needs a different pose origin.
