# Robot Conveyor Action Console

Local Nucleo control and calibration UI for:

- M1 physical-right and M6 physical-left side mechanisms
- M3/M4 conveyor belts
- both LAC targets
- physical-right D13/PA5 ToF feedback
- linked action pairs with exact opposite motor directions
- separate conveyor and LAC box-in/box-out actions
- piecewise-linear M1/M6 clamp profiles over M3/M4 conveyor travel
- cumulative in-editor action demos that move only the target-step difference
- per-action ToF modes (finish range, record-only, or not applicable)
- persistent JSONL command and observation logs
- a large top-bar trial timer that starts on the first saved Conveyor box-out
  after a timer start flag and freezes when the subsequent Conveyor box-in
  completes
- temporary direct positioning that never creates or changes saved actions

Run:

```bash
python3 tools/robot_control/server.py
```

Open `http://127.0.0.1:8768`.

Saved actions are configured only through their pencil controls. The Direct
robot control panel is independent: its stepper and LAC commands are useful for
initial positioning and testing, are logged, and never alter an action.

The side-mechanism directions were physically verified: Clamp is M1 `R` + M6
`R` (inward/towards each other), while Release is M1 `F` + M6 `F`
(outward/away from each other). Built-in action semantics are fixed in both the
editor and backend and cannot be remapped. Step totals are open-loop requested
pulses because the motors have no encoder feedback.

The UI never arms automatically. Board `ARM` stops the motor outputs but always
preserves both existing LAC targets. The UI asks for an area-clear confirmation.
Space or the red STOP button stops all steppers.

Paired step counts use the Nucleo's exact, self-stopping coordinated `C`
command. Each ramp stage is bounded to 100,000 pulses per motor, so a long
action is delivered as several finite board commands instead of one indefinite
continuous signal. The JSONL log records every segment and its delivered pulse
estimate if STOP interrupts a segment.
Saving Clamp/Release, Move left/right, or Box in/out automatically copies the
steps, speed, and ramp to its pair and reverses every motor direction. Each side
keeps its own ToF finish setting. A Conveyor pencil can also define cumulative
extra clamp steps against conveyor progress: positive changes use the verified
Clamp direction (`M1 R + M6 R`), negative changes use Release, equal values hold
the existing opening, and points are joined linearly. Each Conveyor pencil can
keep its clamp curve independent, copy the other action's reverse into the
editor for further tweaks, or explicitly replace the paired curve with an exact
time- and direction-reversed copy on save. Because M1 shares
TIM3 with M3/M4, the host executes this as short synchronized/interleaved slices
that keep M1 and M6 pulse totals matched. These are open-loop position offsets,
not measured force; no load cell is fitted. Every editor demo requests a fresh working
physical-right ToF reading before and after movement and records the result.
The top action bar treats `Conveyor: box in/out` as M3/M4 stepper motions and
`Linear actuator: box in/out` as independent absolute LAC1+LAC2 percentage
commands. They are never emitted as one combined command. Stepper ramps use a
symmetric five-stage approximation of a linear speed profile; the editor graph
shows the actual commanded shape and a backend-calculated total runtime. A
movement is not rejected merely because it takes longer than one minute.
Actions accept 1–10,000,000 commanded steps; long coordinated conveyor/clamp
runs are split into Nucleo-safe commands of at most 100,000 steps per motor.
Speed accepts any whole number from 100–4,000 Hz. The 4,000 Hz maximum remains
because it is the flashed Nucleo coordinated-scheduler safety limit.
Saved actions and session logs are stored in `tools/robot_control/data`.
Use `--data-dir` to select another directory.
