"""Recorded simulation cases: fields worth keeping, to study them and to re-run them as tests.

A case file (avoid_sim/cases/*.yaml) holds a list of cases. A case is a complete field: the
obstacle centres and the start pose are stored explicitly, so it does not depend on the layout
sampler (a seed alone would change meaning whenever the sampler changes). The seed is kept as
the noise seed: it reseeds the simulated odometry and camera, so a case replays the same noise.

  - id: wide-gap-slow                 # unique in the file
    recorded: '2026-11-10 10:30'
    sensor: camera                    # configuration it was seen with; evaluate.py skips others
    seed: 2                           # noise seed (and the layout seed it came from, if any)
    layout: random                    # where the obstacles came from (information only)
    start: [-2.0, 0.0, 0.0]           # x [m], y [m], yaw [deg]
    obstacles: [[-0.31, -0.5], ...]   # centres [m], or [x, y, yaw deg] for a turned box; the size comes
                                      # from /obstacles/size
    note: what went wrong / what to look at
    observed: what happened when it was recorded
    expect: {result: success, max_time: 8.0}
    status: open                      # open: known problem | fixed: regression test

expect keys: result (success | collision | line | timeout | safe_stop), max_time [s],
min_obstacle [m], min_line [m]. Missing keys are not checked; no expect = {result: success}.
"""
import datetime
import math
import os

import yaml

HEADER = """\
# avoid_sim cases: recorded fields, kept to study them and to re-run them as tests.
# Format: avoid_sim/src/avoid_sim/cases.py. Run: rosrun avoid_sim evaluate.py --cases <this file>
# or avoid_bringup/scripts/run_cases.sh. Record from the web UI (simulation: 記錄這個 case).
"""
EXPECT_KEYS = ('result', 'max_time', 'min_obstacle', 'min_line')
STATUSES = ('open', 'fixed')


def cases_dir():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'cases')


def resolve(path):
    """A case file: a path, or a bare name inside avoid_sim/cases."""
    if os.path.sep in path or os.path.exists(path):
        return path
    return os.path.join(cases_dir(), path if path.endswith('.yaml') else path + '.yaml')


def load(path):
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    cases = data.get('cases') or []
    for c in cases:
        validate(c)
    return cases


def validate(c):
    for k in ('id', 'seed', 'start', 'obstacles'):
        if k not in c:
            raise ValueError('case %s: missing %s' % (c.get('id', '?'), k))
    if len(c['start']) != 3:
        raise ValueError('case %s: start must be [x, y, yaw_deg]' % c['id'])
    bad = set(c.get('expect') or {}) - set(EXPECT_KEYS)
    if bad:
        raise ValueError('case %s: unknown expect keys %s' % (c['id'], sorted(bad)))
    if c.get('status', 'open') not in STATUSES:
        raise ValueError('case %s: status must be one of %s' % (c['id'], STATUSES))


def _header_of(path):
    """Leading comment lines of an existing file (kept when cases are appended)."""
    if not os.path.exists(path):
        return HEADER
    lines = []
    with open(path) as fh:
        for line in fh:
            if not line.startswith('#'):
                break
            lines.append(line)
    return ''.join(lines) or HEADER


def append(path, case):
    """Append a case (validated, id made unique) and rewrite the file. Returns the id used."""
    cases = load(path) if os.path.exists(path) else []
    ids = {c['id'] for c in cases}
    base, n = case['id'], 2
    while case['id'] in ids:
        case['id'] = '%s-%d' % (base, n)
        n += 1
    validate(case)
    cases.append(case)
    header = _header_of(path)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as fh:
        fh.write(header)
        yaml.safe_dump({'cases': cases}, fh, allow_unicode=True, sort_keys=False, default_flow_style=None, width=110)
    os.replace(tmp, path)
    return case['id']


def make(seed, layout, start, obstacles, sensor, note='', observed='', expect=None, status='open', case_id=''):
    """start: (x, y, yaw_rad); obstacles: [(x, y)] or [(x, y, yaw_rad)] (the yaw is stored only when not 0)."""
    now = datetime.datetime.now()
    return {
        'id': case_id or 'case-%s-seed%d' % (now.strftime('%Y%m%d-%H%M%S'), seed),
        'recorded': now.strftime('%Y-%m-%d %H:%M'),
        'sensor': sensor or 'camera',
        'seed': int(seed),
        'layout': layout,
        'start': [round(float(start[0]), 4), round(float(start[1]), 4), round(float(start[2]) * 180.0 / 3.141592653589793, 3)],
        'obstacles': [[round(float(o[0]), 4), round(float(o[1]), 4)] +
                      ([round(math.degrees(float(o[2])), 3)] if len(o) > 2 and o[2] else []) for o in obstacles],
        'note': note,
        'observed': observed,
        'expect': dict(expect) if expect else {'result': 'success'},
        'status': status or 'open',
    }
