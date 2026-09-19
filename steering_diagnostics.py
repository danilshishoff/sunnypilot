"""Offline steering diagnostics; never connects to a vehicle or writes CAN.

Accepts the JSON extracts from this analysis, or rlog/rlog.zst with --schema
and --imports (matching cereal/opendbc schema directories). Raw input requires
pycapnp and, for .zst, zstandard. JSON input uses only the Python standard library.
"""
import argparse
import bisect
import json
import math
from pathlib import Path
import statistics


class Stream:
    def __init__(self, rows):
        self.rows = sorted(rows, key=lambda x: x[0])
        self.times = [x[0] for x in self.rows]

    def at(self, time, max_age=50_000_000):
        # Causal alignment: never borrow a value from the future.
        i = bisect.bisect_right(self.times, time) - 1
        if i < 0 or time - self.times[i] > max_age:
            return None
        return self.rows[i]


def spans(rows, predicate):
    """Contiguous observed spans; false/missing frames and gaps end a span."""
    result, current = [], None
    for row in rows:
        flag = predicate(row)
        if flag and current and row['t'] - current['last'] <= .03:
            current['end'] = row['t'] + row['dt']
            current['last'] = row['t']
        else:
            if current:
                result.append(current)
            current = ({'start': row['t'], 'end': row['t'] + row['dt'],
                        'last': row['t']} if flag else None)
    if current:
        result.append(current)
    return [{'start_s': round(x['start'], 3), 'end_s': round(x['end'], 3),
             'duration_s': round(x['end'] - x['start'], 3)} for x in result]


def load_input(path, schema=None, imports=()):
    if path.suffix == '.json':
        return json.loads(path.read_text(encoding='utf-8'))
    if schema is None:
        raise ValueError('Raw rlog requires --schema and matching --imports')
    import capnp
    raw = path.read_bytes()
    if path.suffix == '.zst':
        import zstandard
        import io
        with zstandard.ZstdDecompressor().stream_reader(io.BytesIO(raw)) as reader:
            raw = reader.read()
    log = capnp.load(str(schema), imports=list(imports))
    data = {k: [] for k in ('carState', 'carControl', 'carOutput', 'controlsState',
                            'selfdriveState', 'onroadEvents', 'lateralTorqueParameters')}
    data['meta'] = {}
    # Decode Cap'n Proto framing explicitly; avoid platform-specific file reader issues.
    import struct
    offset = 0
    while offset < len(raw):
        if len(raw) - offset < 8:
            raise ValueError('Truncated Capn Proto frame header')
        count = struct.unpack_from('<I', raw, offset)[0] + 1
        if count > 512:
            raise ValueError('Invalid Capn Proto segment count')
        header = ((count + 2) // 2) * 8
        sizes = struct.unpack_from('<' + 'I' * count, raw, offset + 4)
        size = header + sum(sizes) * 8
        if offset + size > len(raw):
            raise ValueError('Truncated Capn Proto message')
        with log.Event.from_bytes(raw[offset:offset + size]) as event:
            kind, t = event.which(), int(event.logMonoTime)
            if kind == 'initData':
                v = event.initData
                data['meta']['initData'] = {k: str(getattr(v, k)) for k in
                                           ('version', 'gitCommit', 'gitBranch')}
            elif kind == 'carState':
                s = event.carState
                data[kind].append([t, s.vEgo, s.vEgoRaw, s.steeringAngleDeg,
                                  s.steeringRateDeg, s.steeringTorque, s.steeringTorqueEps,
                                  s.steeringPressed, s.gasPressed, s.brakePressed,
                                  s.steerFaultTemporary, s.steerFaultPermanent])
            elif kind == 'carControl':
                s = event.carControl
                data[kind].append([t, s.enabled, s.latActive, s.actuators.torque])
            elif kind == 'carOutput':
                s = event.carOutput.actuatorsOutput
                data[kind].append([t, s.torque, s.steeringAngleDeg, s.curvature, s.torqueOutputCan])
            elif kind == 'controlsState':
                s = event.controlsState
                tag = s.lateralControlState.which()
                v = getattr(s.lateralControlState, tag)
                data[kind].append([t, s.curvature, s.desiredCurvature, tag, v.active,
                                  v.saturated, v.output] + [getattr(v, k, None) for k in
                                  ('actualLateralAccel', 'desiredLateralAccel', 'error',
                                   'steeringAngleDeg', 'steeringAngleDesiredDeg', 'actualCurvature')])
            elif kind == 'selfdriveState':
                s = event.selfdriveState
                data[kind].append([t, s.enabled, s.active, str(s.state), s.alertType,
                                  s.alertText1, s.alertText2])
            elif kind == 'onroadEvents':
                data[kind].append([t, [str(v.name) for v in event.onroadEvents]])
            elif kind == 'lateralTorqueParameters':
                v = event.lateralTorqueParameters
                data[kind].append([t, {k: getattr(v, k) for k in
                    ('valid', 'useParams', 'version', 'latAccelFactorFiltered',
                     'frictionCoefficientFiltered', 'totalBucketPoints', 'calPerc')}])
        offset += size
    return data


def analyze(data, name):
    controls = sorted(data.get('controlsState', []), key=lambda x: x[0])
    car = Stream(data.get('carState', []))
    command = Stream(data.get('carControl', []))
    output = Stream(data.get('carOutput', []))
    if not controls or not car.rows or not command.rows:
        raise ValueError('Missing controlsState, carState or carControl')
    origin = car.times[0]
    pressed_times = [x[0] for x in car.rows if x[7]]
    intervals = [(b[0] - a[0]) / 1e9 for a, b in zip(controls, controls[1:])
                 if 0 < b[0] - a[0] <= 30_000_000]
    step = statistics.median(intervals) if intervals else .01
    rows = []
    for i, c in enumerate(controls):
        t = c[0]
        dt = ((controls[i+1][0] - t) / 1e9 if i+1 < len(controls) else step)
        dt = dt if 0 < dt <= .03 else 0
        s, cc, co = car.at(t), command.at(t), output.at(t)
        row = {'t': (t-origin)/1e9, 'dt': dt, 'usable': False}
        rows.append(row)
        if s is None or cc is None:
            continue
        active = bool(c[4] and cc[2])
        # First 2 s have unknown driver history; conservatively exclude them.
        p = bisect.bisect_right(pressed_times, t) - 1
        clean = t-origin >= 2_000_000_000 and (p < 0 or t-pressed_times[p] >= 2_000_000_000)
        row.update(usable=True, active=active, clean=clean, speed=s[1]*3.6,
                   at_limit=abs(c[6]) >= .999, sign=1 if c[6] >= 0 else -1,
                   saturated=bool(c[5]), pressed=bool(s[7]), angle=s[3],
                   gas=bool(s[8]), brake=bool(s[9]), fault=bool(s[10] or s[11]),
                   can=co[4] if co else None,
                   # Common comparison proxy, not inertial measurement or PID error.
                   accel_error_proxy=abs(c[2]-c[1])*s[1]**2)
        row['pid_angle_error'] = abs(c[11]-c[10]) if c[3] == 'pidState' else None
        row['torque_error'] = abs(c[9]) if c[3] == 'torqueState' else None
    active = [r for r in rows if r.get('active')]
    clean = [r for r in active if r['clean']]
    def duration(items, pred=lambda x: True):
        return round(sum(r['dt'] for r in items if pred(r)), 3)
    bins = []
    for lo, hi in ((0, 10), (10, 20), (20, 36), (36, 50), (50, 80), (80, math.inf)):
        subset = [r for r in clean if lo <= r['speed'] < hi]
        weight = sum(r['dt'] for r in subset)
        if weight:
            bins.append({'speed_kph': f'{lo}-{hi}', 'eligible_s': round(weight, 3),
                         'at_limit_s': duration(subset, lambda r:r['at_limit']),
                         'mean_accel_error_proxy_mps2': round(sum(r['accel_error_proxy']*r['dt'] for r in subset)/weight, 4)})
    episodes = []
    for sign in (-1, 1):
        for ep in spans(rows, lambda r: r.get('active') and r['clean'] and r['at_limit'] and r['sign'] == sign):
            ep['sign'] = sign
            episodes.append(ep)
    sd = sorted(data.get('selfdriveState', []), key=lambda x:x[0])
    alert_rows = []
    for i, x in enumerate(sd):
        dt = (sd[i+1][0]-x[0])/1e9 if i+1<len(sd) else step
        alert_rows.append({'t': (x[0]-origin)/1e9, 'dt': dt if 0<dt<=.03 else 0,
                           'alert': x[4] == 'steerSaturated/warning'})
    return {'segment': name, 'controllers': sorted({x[3] for x in controls}),
            'source_commit': data.get('meta', {}).get('initData', {}).get('gitCommit'),
            'active_s': duration(active), 'eligible_s': duration(clean),
            'at_limit_active_s': duration(active,lambda r:r['at_limit']),
            'at_limit_eligible_s': duration(clean,lambda r:r['at_limit']),
            'saturated_s': duration(active,lambda r:r['saturated']),
            'driver_pressed_s': duration(active,lambda r:r['pressed']),
            'fault_s': duration(active,lambda r:r['fault']),
            'angle_range_deg': [min(x[3] for x in car.rows),max(x[3] for x in car.rows)],
            'missing_alignment_frames': sum(not r['usable'] for r in rows),
            'car_output_max_abs_can': max((abs(x[4]) for x in output.rows), default=None),
            'warnings': spans(alert_rows,lambda r:r['alert']),
            'warning_events_seen': any('steerSaturated' in x[1] for x in data.get('onroadEvents', [])),
            'longest_eligible_limit_episodes': sorted(episodes,key=lambda x:x['duration_s'],reverse=True)[:5],
            'speed_bins': bins,
            'torque_params_first_last': (data.get('lateralTorqueParameters', [])[:1] + data.get('lateralTorqueParameters', [])[-1:]),
            'extraction_errors': data.get('errors', [])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', nargs='+', type=Path)
    parser.add_argument('--schema', type=Path)
    parser.add_argument('--imports', action='append', default=[])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    reports = [analyze(load_input(p,args.schema,args.imports),p.name) for p in args.inputs]
    args.output.write_text(json.dumps(reports,ensure_ascii=False,indent=2),encoding='utf-8')
    for r in reports:
        print(r['segment'], r['controllers'], 'eligible',r['eligible_s'],
              'max command eligible',r['at_limit_eligible_s'], 'saturated',r['saturated_s'],
              'warnings',r['warnings'])


if __name__ == '__main__':
    main()
