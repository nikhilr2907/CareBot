# Lift Calibration Console

Local browser control and human-feedback logging for the two Y-axis steppers:

- M2: Y axis right
- M5: Y axis left
- Up: reverse direction
- Down: forward direction

The corresponding STM32 firmware command is:

```text
LIFT <UP|DOWN> <M2_hz> <M5_hz> <duration_ms>
```

The STM32 enforces a 100-10000 ms duration and stops M2/M5 on the board when
the timer expires. The web console applies tighter calibration limits of
300-4000 Hz and 100-3000 ms.

Run from `firmware/pick_place`:

```bash
python3 tools/lift_calibrator/server.py --port 8767
```

The launcher `start_lift_calibrator.command` runs the same server and forwards
additional arguments. The server binds to `127.0.0.1:8767` by default.
Session events and the learned policy are stored in `tools/lift_calibrator/data`.
Use `--data-dir` to select another directory and `--no-browser` for headless use.

Keyboard controls:

- Up/Down arrows: start the selected timed trial
- Space: stop all steppers
- A: left stalled
- D: right stalled
- S: smooth
- B: both stalled
- V: vibration

The adaptive rule never raises either motor above the selected base speed.
When one side stalls, the next recommendation slows the opposite side by
2.5%. Vibration or a two-sided stall reduces the base speed by 200 Hz.

Use `--simulate` to exercise the interface without opening a serial device.

Run the focused policy tests with:

```bash
cd tools/lift_calibrator
python3 -m unittest -v test_server.py
```
