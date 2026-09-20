"""Receive-only Jetta Torque-v1 shadow experiment. Never publishes vehicle commands."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import subprocess
import time

SOURCE_COMMIT = '6a17f75c6bcb67c85f252a1acc342d94d5b8a4d2'
STOCK_UNITS = 300
SHADOW_UNITS = 500


class PastSamples:
  """Keep received messages until the control timestamp catches up with them."""
  def __init__(self, names):
    self.history = {name: deque(maxlen=128) for name in names}

  def add(self, name, timestamp, value, valid):
    self.history[name].append((timestamp, value, valid))

  def at(self, name, timestamp):
    return next((sample for sample in reversed(self.history[name]) if sample[0] <= timestamp), None)


class ShadowPair:
  """Two independently constructed controllers; outputs only become plain log data."""
  def __init__(self, factory):
    self.baseline = factory()
    self.shadow = factory()
    if self.baseline is self.shadow or self.baseline.pid is self.shadow.pid:
      raise ValueError('Independent controller state required')
    self.baseline.steer_max = 1.0
    self.shadow.steer_max = SHADOW_UNITS / STOCK_UNITS
    self.baseline.update_limits()
    self.shadow.update_limits()
    self.frames = 0

  def reset(self):
    # Rebuild on dropped input in the runner: reset() alone does not clear
    # the upstream torque controller's request buffer or jerk filter.
    self.baseline.reset()
    self.shadow.reset()
    self.frames = 0

  def update(self, active, args, observed, torque_params=None):
    if not active:
      self.reset()
    result = {}
    for name, controller in (('baseline', self.baseline), ('shadow', self.shadow)):
      if torque_params is not None:
        controller.update_live_torque_params(*torque_params)
      output, _, state = controller.update(active, *args)
      if not math.isfinite(output) or abs(output) > controller.steer_max + 1e-4:
        raise ValueError('Non-finite or out-of-bound shadow calculation')
      result[name + '_units'] = float(output * STOCK_UNITS)
      result[name + '_saturated'] = bool(state.saturated)
      result[name + '_i'] = float(controller.pid.i)
    self.frames = self.frames + 1 if active else 0
    result.update(active=bool(active), warmup=self.frames < 200,
                  observed_units=float(observed * STOCK_UNITS),
                  baseline_difference_units=float(result['baseline_units'] - observed * STOCK_UNITS))
    return result


def run(duration, destination):
  # Imports intentionally delayed so the isolation/math tests run on a PC.
  from cereal import car, custom
  import cereal.messaging as messaging
  from openpilot.common.params import Params
  from opendbc.car.car_helpers import interfaces
  from opendbc.car.vehicle_model import VehicleModel
  from opendbc.car.volkswagen.values import CarControllerParams
  from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
  from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS

  root = Path(__file__).resolve().parent
  revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
  # This additive branch may have commits on top of the pinned release, but
  # all tracked upstream files must remain byte-identical to that release.
  changed = subprocess.check_output(['git', '-C', str(root), 'diff', '--name-only', SOURCE_COMMIT, '--'], text=True).splitlines()
  allowed = {'steering_shadow.py', 'test_steering_shadow.py', 'SHADOW_README.md'}
  if set(changed) - allowed:
    raise RuntimeError('Upstream source differs from pinned release; refusing to run')
  params = Params()
  cp_bytes, sp_bytes = params.get('CarParams'), params.get('CarParamsSP')
  if not cp_bytes or not sp_bytes:
    raise RuntimeError('CarParams/CarParamsSP unavailable; wait for normal vehicle identification')

  def decode():
    return (messaging.log_from_bytes(cp_bytes, car.CarParams),
            messaging.log_from_bytes(sp_bytes, custom.CarParamsSP))

  cp, _ = decode()
  if cp.carFingerprint != 'VOLKSWAGEN_JETTA_MK7' or cp.lateralTuning.which() != 'torque':
    raise RuntimeError('Only Jetta MK7 with Torque control is supported')
  if CarControllerParams.STEER_MAX != STOCK_UNITS:
    raise RuntimeError('Stock vehicle limit is not 300')

  def check_settings():
    if not params.get_bool('EnforceTorqueControl') or params.get_bool('NeuralNetworkLateralControl'):
      raise RuntimeError('Requires enforced Torque v1 with NNLC off')
    if float(params.get('TorqueControlTune')) != 1.0:
      raise RuntimeError('Requires TorqueControlTune v1')

  check_settings()

  def factory():
    local_cp, local_sp = decode()
    ci = interfaces[local_cp.carFingerprint](local_cp, local_sp)
    return LatControlTorque(local_cp, local_sp, ci, .01)

  pair = ShadowPair(factory)
  vm = VehicleModel(cp)
  services = ['controlsState', 'carControl', 'carState', 'carOutput', 'liveParameters',
              'liveTorqueParameters', 'liveDelay']
  # Receive-only: no PubMaster, CAN socket, Controls instance, or actuator writer.
  sm = messaging.SubMaster(services, poll='controlsState')
  history = PastSamples(services)
  started = time.monotonic()
  previous_t = None
  previous_real_command = 0.0
  safety_limited = False
  last_settings_check = started
  with destination.open('x', encoding='utf-8', buffering=1) as output_file:
    output_file.write(json.dumps({'type': 'metadata', 'commit': revision, 'base': SOURCE_COMMIT,
      'stock_limit': STOCK_UNITS, 'shadow_limit': SHADOW_UNITS,
      'note': 'Observed-trajectory shadow; no hypothetical EPS or vehicle response'}) + '\n')
    while time.monotonic() - started < duration:
      sm.update(100)
      for service in services:
        if sm.updated[service]:
          history.add(service, sm.logMonoTime[service], sm[service], sm.valid[service])
      now = time.monotonic()
      if now - last_settings_check >= 1:
        check_settings()
        last_settings_check = now
      if not sm.updated['controlsState']:
        continue
      t = sm.logMonoTime['controlsState']
      selected = {s: history.at(s, t) for s in services}
      ages = {s: (t - sample[0]) / 1e9 if sample else None for s, sample in selected.items()}
      # Never use a future sample; retain it for the next applicable timestamp.
      fast = ['carState', 'carControl', 'carOutput']
      aligned = all(ages[s] is not None and 0 <= ages[s] <= .03 for s in fast)
      aligned &= all(ages[s] is not None and 0 <= ages[s] <= 2 for s in ['liveParameters', 'liveDelay'])
      valid = all(selected[s] is not None and selected[s][2] for s in services if s != 'liveTorqueParameters')
      gap = previous_t is not None and not .005 <= (t - previous_t) / 1e9 <= .02
      previous_t = t
      if not aligned or not valid or gap:
        pair = ShadowPair(factory)
        safety_limited = False
        previous_real_command = float(sm['carControl'].actuators.torque)
        output_file.write(json.dumps({'type': 'skip', 't': t, 'ages': ages, 'gap': gap, 'valid': valid}) + '\n')
        continue
      values = {s: sample[1] for s, sample in selected.items() if sample}
      state = values['controlsState']
      if state.lateralControlState.which() != 'torqueState' or state.lateralControlState.torqueState.version != 1:
        raise RuntimeError('Observed controller is not Torque v1')
      cs, cc, lp = values['carState'], values['carControl'], values['liveParameters']
      vm.update_params(max(lp.stiffnessFactor, .1), max(lp.steerRatio, .1))
      tp = values.get('liveTorqueParameters')
      tune = ((tp.latAccelFactorFiltered, tp.latAccelOffsetFiltered, tp.frictionCoefficientFiltered)
              if tp is not None and selected['liveTorqueParameters'][2] and ages['liveTorqueParameters'] <= 2 and tp.useParams else None)
      # Both twins use the observed safety feedback, deliberately conservative.
      # This does not simulate panda rate limiting or a 500-unit actuator.
      args = (cs, vm, lp, safety_limited, state.desiredCurvature, None, False,
              values['liveDelay'].lateralDelay + LAT_SMOOTH_SECONDS)
      row = pair.update(cc.latActive, args, state.lateralControlState.torqueState.output, tune)
      row.update(type='sample', t=t, speed_mps=float(cs.vEgo), angle_deg=float(cs.steeringAngleDeg),
                 steering_pressed=bool(cs.steeringPressed), gas_pressed=bool(cs.gasPressed),
                 brake_pressed=bool(cs.brakePressed), desired_curvature=float(state.desiredCurvature),
                 actual_curvature=float(state.curvature), observed_safety_limited=safety_limited,
                 car_output_units=int(values['carOutput'].actuatorsOutput.torqueOutputCan), ages=ages)
      output_file.write(json.dumps(row, allow_nan=False) + '\n')
      safety_limited = abs(previous_real_command - values['carOutput'].actuatorsOutput.torque) > .01
      previous_real_command = float(cc.actuators.torque)
      if output_file.tell() > 100_000_000:
        raise RuntimeError('100 MB log size cap reached')


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--seconds', type=int, default=120, choices=range(1, 601), metavar='1..600')
  args = parser.parse_args()
  # Lower priority than the actual driving processes. No realtime scheduling.
  if hasattr(os, 'nice'):
    os.nice(15)
  run(args.seconds, args.output)


if __name__ == '__main__':
  main()
