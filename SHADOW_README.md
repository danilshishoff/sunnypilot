# Receive-only 300/500 shadow prototype

Base: sunnypilot release-mici 2026.002.002,
`6a17f75c6bcb67c85f252a1acc342d94d5b8a4d2`.

This is a standalone, manually started receiver, NOT an install-ready driving
build. No controlsd, manager, carcontroller or panda modifications are included.
It never publishes any messages. The vehicle continues receiving stock commands.

Two separate Torque-v1 controllers calculate on the observed vehicle trajectory.
The baseline has normalized limit 1; the experimental copy has limit 500/300.
Their output is expressed in equivalent command units, not measured EPS torque.
Each controller has its own integrator and history. This is not multiplication
of a clipped 300-unit command. Only Jetta MK7, enforced Torque v1, NNLC disabled
is supported; other configurations stop with an error.

## Status and limitations

Prototype; host-side unit tests only. Full sunnypilot runtime integration and
device resource usage have NOT yet been verified. Do not treat it as ready for
a driving test. First validate with recorded-message replay on the pinned runtime.

No hypothetical vehicle response, CAN rate limiting, EPS acceptance or increased
steering performance is simulated. Both copies retain observed safety feedback.
Curvature-limit flag is unavailable to the receiver and supplied as false; this
affects the saturation diagnostic. NNLC is excluded so pose/model inputs are not
used. Async samples are only approximately aligned. Future messages are buffered
until their timestamps are applicable. Missing/stale inputs and gaps are logged
as skips and rebuild both controllers. Baseline disagreement with
the real controller must be investigated BEFORE interpreting the 500 comparison.
Warmup is at least 200 consecutive active samples and does not prove equivalence.
Manual parameter reads may occur at a different phase from the real controller.

## Validation

On a PC, Python standard library is enough for the isolation/limit tests:

```
python -m unittest discover -s . -p test_steering_shadow.py
```

In a complete pinned sunnypilot runtime, with recorded services being replayed,
from the repository root (choose a new output filename):

```
python steering_shadow.py --seconds 120 --output /tmp/steering-shadow.jsonl
```

The program checks upstream tracked files against the pinned revision, reads
CarParams/CarParamsSP, subscribes to existing services, and writes a local JSONL
file. It never modifies parameters. It runs at lower scheduling priority and
stops after 1–600 seconds or 100 MB. An existing output file is never overwritten.
No automatic startup, installer URL, or change to the vehicle limit is provided.
Personal log files must remain local unless explicitly shared by their owner.
