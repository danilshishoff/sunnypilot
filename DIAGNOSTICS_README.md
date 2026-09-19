# Offline steering diagnostics

This tool processes saved logs on a computer. It does not connect to a vehicle,
write CAN, alter a controller, change panda safety, or install on a comma device.
All existing actuator limits are unchanged.

## Usage

Python 3.10 or newer. The JSON input mode uses the standard library only:

```sh
python steering_diagnostics.py segment22.json segment32.json --output comparison.json
python -m unittest discover -s . -p test_steering_diagnostics.py
```

JSON must follow the positional extract format used by this tool's raw decoder;
arbitrary JSON exports are not accepted. Missing required streams raise an error.

For original rlog or rlog.zst input, install pycapnp and zstandard, and supply
the cereal log.capnp and imports from the exact recorded software revision:

```sh
python steering_diagnostics.py route--4--rlog.zst route--5--rlog.zst \
  --schema path/to/cereal/log.capnp \
  --imports path/to/cereal \
  --imports path/to/opendbc/car \
  --output comparison.json
```

The decoder has been exercised with the schemas available during this analysis;
always check the recorded revision and schema compatibility for another release.
On Windows, use relative schema/import paths if pycapnp fails on Unicode paths.

## Interpretation

- `at_limit_*`: absolute controller output >= 0.999, independently of its
  `saturated` flag or the displayed warning. Max-output runs split on a sign
  change, a false frame, or a data gap. They are not automatically EPS faults.
- `saturated_s`: the controller's logged saturation flag. Its internal gates
  may suppress this flag at low speed or during driver intervention.
- `warnings`: observed `steerSaturated/warning` display intervals; the display
  can persist after the underlying event clears.
- `eligible_s`: active lateral control with aligned signals and at least two
  seconds since the logged steeringPressed flag. Excludes the first two
  seconds (unknown prior history). This is an analytical filter, not a request
  to remove hands from the wheel, and does not prove absence of driver torque.
- Streams align to the most recent past message within 50 ms. This is not
  exact control-cycle synchronization; missing matches are counted.
- Durations use timestamps rather than assuming every frame is exactly 10 ms.
  The final sample uses a median observed sample period. A gap >30 ms is not
  counted as valid duration.
- Speed-bin error is abs(desiredCurvature - curvature) * vEgo^2. It is a
  vehicle-model proxy shared by PID/Torque, not direct inertial acceleration
  and not the torque controller's compensated error. At very low speed or
  extreme steering angles its physical accuracy is limited.
- carOutput is the software controller's reported output, not independent
  proof of panda transmission, EPS acceptance or measured motor torque.
- Live filtered parameters may be manually supplied: `useParams=true` with
  `valid=false` does not itself mean successful self-tuning.

Different routes, speeds, driver inputs and requested curvature cannot establish
which tune is better. For a practical A/B comparison, keep the same controller
baseline, model, vehicle setup and intended trajectory, change one supported
setting at a time while parked, and record normal supervised operation. Keep
hands ready to take over; do not induce an unsafe trajectory to improve a log.
If steering becomes erratic or unexpected, discontinue the run. Saved-log
analysis cannot reproduce the vehicle response to a different controller.

The repository contains code/tests/documentation only. Driving logs and derived
personal reports should remain local unless their owner explicitly shares them.
