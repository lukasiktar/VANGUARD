#!/usr/bin/env python3
"""vg: start / stop the VANGUARD stack (ROS 2 container + mapper container) from one place.

Runs on the Jetson. Every stage runs detached inside its container, in its own process group,
with its output in .vg/logs/<stage>.log, so it keeps running when the SSH session drops.
Settings come from vg.yaml (see there). The existing scripts are only called, never changed.

  ./vg.py modes                          modes and their stages
  ./vg.py up <mode|stage>... [--map M] [--set key=value]... [--dry-run]
  ./vg.py status                         what is running
  ./vg.py stop <stage>...                stop stages (SIGINT to the group, like Ctrl-C)
  ./vg.py down [--containers]            stop everything (and the mapper container)
  ./vg.py logs <stage> [-f] [-n N]
  ./vg.py check [-d SECONDS]             topic rates, who publishes to PX4, cross_rel agreement
  ./vg.py record [name] / ./vg.py stop record
  ./vg.py analyze <bag name> [t0 t1]     compare_cross_rel.py extract + analyze
  ./vg.py maps                           saved maps
  ./vg.py config [mode]                  resolved settings
"""
import argparse
import datetime
import os
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
MAPPER = ROOT / 'mapper'
LOGS = ROOT / '.vg' / 'logs'
PX4_IN = '/fmu/in/vehicle_visual_odometry'

CONTAINERS = {
    'ros2': dict(
        name='vanguard_ros2', user=None, cwd='/ros2_ws', shell='-lc',
        env='source /opt/ros/$ROS_DISTRO/setup.bash; '
            '[ -f install/setup.bash ] && source install/setup.bash; '),
    'mapper': dict(
        name=f'isaac_ros_dev-{os.uname().machine}-d455-container', user='admin',
        cwd='/workspaces/isaac_ros-dev', shell='-c',
        env='source /opt/ros/humble/setup.bash; '
            '[ -f install/setup.bash ] && source install/setup.bash; '),
}

# Canonical start order; `down` stops in reverse.
ORDER = ['agent', 'optitrack', 'mocap_to_px4', 'mocap_gt', 'cross_rel_gt', 'mapper', 'map',
         'localize', 'vslam_to_px4', 'cross_rel_base', 'rviz', 'record']
# Never at the same time: both feed EKF2 external vision / both need the camera.
CONFLICTS = [('mocap_to_px4', 'vslam_to_px4'), ('map', 'localize')]
NEEDS_MAPPER = {'map', 'localize', 'cross_rel_base', 'rviz'}
NAME_RE = re.compile(r'^[A-Za-z0-9_.-]+$')


def die(msg):
    print(f'vg: {msg}', file=sys.stderr)
    sys.exit(1)


def info(msg):
    print(f'vg: {msg}', flush=True)


# ---------------------------------------------------------------- config

def set_dotted(cfg, key, value):
    d = cfg
    parts = key.split('.')
    for p in parts[:-1]:
        d = d.setdefault(p, {})
    d[parts[-1]] = value


def get_dotted(cfg, key):
    d = cfg
    for p in key.split('.'):
        if not isinstance(d, dict) or p not in d:
            raise KeyError(key)
        d = d[p]
    return d


def load_config(args, modes_named=()):
    path = Path(args.config)
    if not path.is_file():
        die(f'{path} not found')
    cfg = yaml.safe_load(path.read_text()) or {}
    for m in modes_named:
        for k, v in (cfg['modes'][m].get('set') or {}).items():
            set_dotted(cfg, k, v)
    for kv in getattr(args, 'set', None) or []:
        if '=' not in kv:
            die(f'--set needs key=value, got {kv!r}')
        k, v = kv.split('=', 1)
        set_dotted(cfg, k.strip(), yaml.safe_load(v))
    if getattr(args, 'map', None):
        cfg['map'] = args.map
    if not NAME_RE.match(str(cfg.get('map', ''))):
        die(f"map name {cfg.get('map')!r}: use letters, digits, _ . - only")
    if cfg.get('platform') not in ('orin_nano', 'laptop'):
        die(f"platform must be orin_nano or laptop, got {cfg.get('platform')!r}")
    return cfg


def ros_value(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, (list, tuple)):
        return '[' + ','.join(ros_value(x) for x in v) + ']'
    return str(v)


def ros_params(params):
    out = [f'-p {shlex.quote(f"{k}:={ros_value(v)}")}' for k, v in params.items() if v is not None]
    return (' --ros-args ' + ' '.join(out)) if out else ''


def mapper_launch_args(cfg, localize):
    m = cfg['mapper']
    a = []
    if m.get('cross_detector'):
        a.append('with_cross_detector:=true')
    if m.get('px4_vio'):
        a.append('px4_vio:=true')
    if localize and m.get('with_pins'):
        a.append('with_pins:=true')
    a += [str(x) for x in m.get('extra_args') or []]
    return ' '.join(shlex.quote(x) for x in a)


# ---------------------------------------------------------------- stages

class Stage:
    """kind: proc (long-running, detached in a container), mapper, rviz."""

    def __init__(self, name, kind, container=None, cmd=None, grace=10, post=None, note=None,
                 skip=None):
        self.name, self.kind, self.container, self.cmd = name, kind, container, cmd
        self.grace, self.post, self.note, self.skip = grace, post, note, skip


ACTIVATE = (
    'n=/mocap4r2_optitrack_driver_node; for i in $(seq 30); do '
    's=$(ros2 lifecycle get $n 2>/dev/null); case "$s" in '
    'active*) echo "$n active"; exit 0;; '
    'inactive*) ros2 lifecycle set $n activate >/dev/null;; esac; sleep 1; done; '
    'echo "$n did not become active (Motive streaming? driver log: ./vg.py logs optitrack)"; exit 1')


def stage(name, cfg, mode_stages=()):
    o, m = cfg['optitrack'], cfg['mapper']
    mp, plat = cfg['map'], cfg['platform']
    if name == 'agent':
        return Stage(name, 'proc', 'ros2', f"MicroXRCEAgent udp4 -p {int(cfg['agent']['port'])}")
    if name == 'optitrack':
        return Stage(name, 'proc', 'ros2',
                     'ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py', post=ACTIVATE)
    if name == 'mocap_to_px4':
        return Stage(name, 'proc', 'ros2', 'ros2 run vanguard mocap_to_px4_node'
                     + ros_params({'body_id': o['drone_id']}),
                     note='OptiTrack -> EKF2. PX4 needs the param_OPTITRACK_* EKF2 set.')
    if name == 'mocap_gt':
        return Stage(name, 'proc', 'ros2', 'ros2 run vanguard mocap_gt_node'
                     + ros_params({'body_id': o['drone_id']}))
    if name == 'cross_rel_gt':
        return Stage(name, 'proc', 'ros2', 'ros2 run vanguard cross_rel_gt' + ros_params({
            'drone_id': o['drone_id'], 'pole_id': o['pole_id'],
            'base_in_pivot_xyz': [float(x) for x in o['base_in_pivot_xyz']],
            'cross_in_pole_xyz': [float(x) for x in o['cross_in_pole_xyz']]}))
    if name == 'mapper':
        return Stage(name, 'mapper')
    if name == 'map':
        skip = None
        if (MAPPER / 'maps' / f'{mp}.db').exists():
            skip = (f'mapper/maps/{mp}.db exists and map.sh would refuse (or delete it with '
                    '--force). Pick a new name: --map <new_name>')
        return Stage(name, 'proc', 'mapper',
                     f'scripts/map.sh {mp} {plat} {mapper_launch_args(cfg, False)}',
                     grace=90, skip=skip, note='Stop with ./vg.py stop map: RTAB-Map saves the '
                     'database on SIGINT (up to 90 s).')
    if name == 'localize':
        skip = None
        if not (MAPPER / 'maps' / f'{mp}.db').exists():
            skip = f'mapper/maps/{mp}.db not found (./vg.py maps)'
        prior = '--prior-map' if m.get('prior_map') else '--no-prior-map'
        return Stage(name, 'proc', 'mapper',
                     f'scripts/localize.sh {mp} {plat} {prior} {mapper_launch_args(cfg, True)}',
                     grace=30, skip=skip)
    if name == 'vslam_to_px4':
        skip = None if m.get('px4_vio') else 'mapper.px4_vio is false, no /vslam/odom_px4'
        return Stage(name, 'proc', 'ros2', 'ros2 run vanguard vslam_to_px4_node'
                     + ros_params(cfg.get('vslam_to_px4') or {}), skip=skip,
                     note='cuVSLAM -> EKF2. PX4 needs the param_CUVSLAM_* EKF2 set.')
    if name == 'cross_rel_base':
        pins = MAPPER / 'maps' / f'{mp}_pins.json'
        skip = None
        if not pins.exists():
            skip = (f'{pins.relative_to(ROOT)} not there yet: place the pin in RViz '
                    '(Publish Point), then ./vg.py up cross_rel_base')
        p = {'pins_file': f'/maps/{mp}_pins.json',
             'pin_id': (cfg.get('cross_rel_base') or {}).get('pin_id')}
        return Stage(name, 'proc', 'mapper', 'ros2 run d455_mapping cross_rel_base'
                     + ros_params(p), skip=skip)
    if name == 'rviz':
        mapping = 'map' in mode_stages or (not mode_stages and 'map' in running_stages())
        return Stage(name, 'rviz', 'mapper',
                     'mapping_remote' if mapping else 'localization_remote')
    if name == 'record':
        topics = ' '.join(shlex.quote(t) for t in cfg['record']['topics'])
        return Stage(name, 'proc', 'ros2',
                     f"ros2 bag record -s sqlite3 -o bags/{cfg['_bag']} {topics}", grace=15)
    custom = (cfg.get('custom_stages') or {}).get(name)
    if custom:
        if custom.get('container') not in CONTAINERS:
            die(f'custom stage {name}: container must be ros2 or mapper')

        def fill(mt):
            try:
                return str(get_dotted(cfg, mt.group(1)))
            except KeyError:
                die(f'custom stage {name}: {{{mt.group(1)}}} is not in the config')
        return Stage(name, 'proc', custom['container'],
                     re.sub(r'\{([\w.]+)\}', fill, custom['cmd']),
                     grace=int(custom.get('grace', 10)))
    die(f"unknown mode or stage '{name}' (./vg.py modes)")


def all_stage_names(cfg):
    return ORDER + [s for s in (cfg.get('custom_stages') or {}) if s not in ORDER]


# ---------------------------------------------------------------- docker helpers

def run(argv, check=True, capture=False, quiet=False):
    r = subprocess.run(argv, text=True, stdout=subprocess.PIPE if capture else None,
                       stderr=subprocess.STDOUT if capture else None)
    if check and r.returncode != 0:
        if capture and not quiet:
            print(r.stdout, file=sys.stderr)
        die(f'failed ({r.returncode}): {" ".join(map(str, argv))}')
    return r


def container_up(key):
    r = subprocess.run(['docker', 'ps', '-q', '--filter', f"name=^{CONTAINERS[key]['name']}$",
                        '--filter', 'status=running'], text=True, capture_output=True)
    return bool(r.stdout.strip())


def dexec(key, script, user=True, capture=True, check=False):
    c = CONTAINERS[key]
    argv = ['docker', 'exec']
    if user and c['user']:
        argv += ['-u', c['user']]
    argv += ['-w', c['cwd'], c['name'], 'bash', c['shell'], script]
    return run(argv, check=check, capture=capture)


def ensure_container(key, dry):
    if container_up(key):
        return
    if dry:
        info(f"(would start container {CONTAINERS[key]['name']})")
        return
    if key == 'ros2':
        info('starting vanguard_ros2 (./ros2-docker.sh up)')
        run([str(ROOT / 'ros2-docker.sh'), 'up'])
    else:
        start_mapper(dry)


def start_mapper(dry):
    steps = []
    if not container_up('mapper'):
        steps.append([str(MAPPER / 'scripts' / 'dev.sh'), '--detach'])
    steps.append([str(MAPPER / 'scripts' / 'usb_perms.sh')])
    for s in steps:
        info(('(dry) ' if dry else '') + ' '.join(s))
        if not dry:
            run(s)


PIDS_SCRIPT = ('for f in /tmp/vg/*.pid; do [ -e "$f" ] || continue; p=$(cat "$f"); '
               'n=$(basename "$f" .pid); if kill -0 -- -"$p" 2>/dev/null; then echo "$n"; '
               'else rm -f "$f"; fi; done')


def running_stages():
    """{stage: container key} of everything vg started that is still alive."""
    out = {}
    for key in CONTAINERS:
        if not container_up(key):
            continue
        r = dexec(key, PIDS_SCRIPT)
        for line in r.stdout.split():
            out[line] = key
        if key == 'mapper' and dexec(key, 'pgrep -x rviz2 >/dev/null').returncode == 0:
            out['rviz'] = key
    if container_up('mapper'):
        out['mapper'] = 'mapper'
    return out


def ev_publishers():
    """Node names publishing to PX4's external-vision input (also ones vg did not start)."""
    if not container_up('ros2'):
        return []
    r = dexec('ros2', f'ros2 topic info -v {PX4_IN} 2>/dev/null')
    nodes, name = [], ''
    for line in r.stdout.splitlines():
        line = line.strip()
        if line.startswith('Node name:'):
            name = line.split(':', 1)[1].strip()
        elif line.startswith('Endpoint type:'):
            if 'PUBLISHER' in line:
                nodes.append(name)
    return nodes


# ---------------------------------------------------------------- start / stop

def log_tail(stage_name, n=15):
    f = LOGS / f'{stage_name}.log'
    if f.exists():
        lines = f.read_text(errors='replace').splitlines()[-n:]
        print('\n'.join('    | ' + ln for ln in lines))


def start_proc(st, dry):
    c = CONTAINERS[st.container]
    inner = (f'echo $$ > /tmp/vg/{st.name}.pid; export PYTHONUNBUFFERED=1; '
             f'{c["env"]}exec {st.cmd}')
    argv = ['docker', 'exec']
    if c['user']:
        argv += ['-u', c['user']]
    argv += ['-w', c['cwd'], c['name'], 'bash', '-c',
             'mkdir -p /tmp/vg && exec setsid -w bash ' + c['shell'] + ' "$0"', inner]
    info(f'{st.name}: [{st.container}] {st.cmd}')
    if dry:
        return True
    LOGS.mkdir(parents=True, exist_ok=True)
    log = open(LOGS / f'{st.name}.log', 'a')
    log.write(f'\n===== vg start {datetime.datetime.now():%Y-%m-%d %H:%M:%S}: {st.cmd}\n')
    log.flush()
    proc = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True)
    log.close()
    for _ in range(6):
        time.sleep(0.5)
        if proc.poll() is not None:
            info(f'{st.name} exited at once ({proc.returncode}). Log (.vg/logs/{st.name}.log):')
            log_tail(st.name)
            return False
    if st.post:
        r = dexec(st.container, c['env'] + st.post)
        print('    ' + r.stdout.strip().replace('\n', '\n    '))
        if r.returncode != 0:
            return False
    if st.note:
        info(f'  note: {st.note}')
    return True


def stop_proc(name, key, grace):
    script = (f'f=/tmp/vg/{name}.pid; [ -e "$f" ] || exit 0; p=$(cat "$f"); '
              f'kill -INT -- -"$p" 2>/dev/null; '
              f'for i in $(seq {grace * 2}); do kill -0 -- -"$p" 2>/dev/null || '
              f'{{ rm -f "$f"; echo stopped; exit 0; }}; sleep 0.5; done; '
              f'kill -KILL -- -"$p" 2>/dev/null; rm -f "$f"; echo "killed after {grace} s"')
    info(f'stopping {name} (up to {grace} s)')
    r = dexec(key, script)
    print('    ' + (r.stdout.strip() or 'stopped'))


def stop_stage(name, cfg, running):
    if name not in running:
        info(f'{name}: not running')
        return
    if name == 'rviz':
        run([str(MAPPER / 'scripts' / 'rviz_vnc.sh'), 'stop'], check=False)
        return
    if name == 'mapper':
        return              # container: only `down --containers` stops it
    grace = 10
    try:
        grace = stage(name, {**cfg, '_bag': 'x'}).grace
    except SystemExit:
        pass
    stop_proc(name, running[name], grace)


# ---------------------------------------------------------------- commands

def expand(targets, cfg):
    """targets (modes and/or stages) -> (ordered stage names, mode names)."""
    stages, modes = [], []
    for t in targets:
        if t in (cfg.get('modes') or {}):
            modes.append(t)
            stages += [s for s in cfg['modes'][t]['stages'] if s not in stages]
        elif t not in stages:
            stages.append(t)
    return stages, modes


def cmd_up(args):
    raw = yaml.safe_load(Path(args.config).read_text())
    stages, modes = expand(args.targets, raw)
    cfg = load_config(args, modes)
    cfg['_bag'] = 'unused'
    explicit = set(args.targets) - set(modes)
    if not cfg.get('rviz', True):
        stages = [s for s in stages if s != 'rviz' or s in explicit]
    if any(s in NEEDS_MAPPER for s in stages) and 'mapper' not in stages:
        stages.insert(0, 'mapper')
    if any(stage(s, cfg, stages).container == 'ros2' for s in stages if s != 'mapper'):
        ensure_container('ros2', args.dry_run)

    running = running_stages()
    for a, b in CONFLICTS:
        for x, y in ((a, b), (b, a)):
            if x in stages and y in running:
                die(f'{x} cannot run together with {y}, which is running. ./vg.py stop {y} first.')
        if a in stages and b in stages:
            die(f'{a} and {b} cannot run together')
    if {'mocap_to_px4', 'vslam_to_px4'} & set(stages) and not args.dry_run:
        mine = {'mocap_to_px4', 'vslam_to_px4'} & set(running)
        others = [n for n in ev_publishers()
                  if not any(n.endswith(x) for x in ('mocap_to_px4', 'vslam_to_px4'))]
        if others and not mine:
            die(f'{PX4_IN} already has publishers not started by vg: {others}. Stop them first.')

    for name in ('map', 'localize'):
        if name in stages and name not in running and stage(name, cfg, stages).skip:
            die(f'{name}: {stage(name, cfg, stages).skip}. Nothing started.')

    info(f"map {cfg['map']}, platform {cfg['platform']}" + (f", mode {', '.join(modes)}"
                                                         if modes else ''))
    for name in stages:
        st = stage(name, cfg, stages)
        if st.kind == 'mapper':
            start_mapper(args.dry_run)
            continue
        if st.kind == 'rviz':       # (re)start: rviz_vnc.sh replaces a running RViz
            argv = [str(MAPPER / 'scripts' / 'rviz_vnc.sh'), 'start', st.cmd]
            info(('(dry) ' if args.dry_run else '') + 'rviz: ' + ' '.join(argv[1:]))
            if not args.dry_run:
                run(argv, check=False)
            continue
        if name in running:
            info(f'{name}: already running')
            continue
        if st.skip:
            info(f'{name}: SKIPPED, {st.skip}')
            continue
        if not start_proc(st, args.dry_run):
            die(f'{name} failed; stopped here. Fix it, then run the same command again '
                '(running stages are skipped).')
    if not args.dry_run:
        info('done. ./vg.py status, ./vg.py logs <stage> -f, ./vg.py check')


def cmd_stop(args):
    cfg = load_config(args)
    running = running_stages()
    for name in args.stages:
        stop_stage(name, cfg, running)


def cmd_down(args):
    cfg = load_config(args)
    running = running_stages()
    for name in reversed(all_stage_names(cfg)):
        if name in running and name != 'mapper':
            stop_stage(name, cfg, running)
    leftovers = [n for n in running if n not in all_stage_names(cfg)]
    for n in leftovers:
        stop_proc(n, running[n], 10)
    if args.containers and container_up('mapper'):
        run([str(MAPPER / 'scripts' / 'dev.sh'), '--stop'], check=False)
    info('all vg stages stopped' + (' and the mapper container' if args.containers else ''))


def cmd_status(args):
    cfg = load_config(args)
    running = running_stages()
    for key, c in CONTAINERS.items():
        print(f"  container {c['name']:<36} {'up' if container_up(key) else 'down'}")
    print()
    for name in all_stage_names(cfg):
        if name == 'mapper':
            continue
        mark = 'RUNNING' if name in running else '-'
        print(f'  {name:<16} {mark}')
    ev = {'mocap_to_px4', 'vslam_to_px4'} & set(running)
    print()
    print(f"  EKF2 external vision source: {', '.join(ev) if ev else 'none started by vg'}")
    if len(ev) > 1:
        print('  !! BOTH sources publish to ' + PX4_IN + ': stop one now')


def cmd_logs(args):
    f = LOGS / f'{args.stage}.log'
    if not f.exists():
        die(f'no log for {args.stage} ({LOGS})')
    os.execvp('tail', ['tail', '-n', str(args.n)] + (['-F'] if args.follow else []) + [str(f)])


def cmd_check(args):
    cfg = load_config(args)
    if not container_up('ros2'):
        die('vanguard_ros2 is not running')
    topics = ' '.join(shlex.quote(t) for t in cfg['check']['topics'])
    c = CONTAINERS['ros2']
    os.execvp('docker', ['docker', 'exec', '-w', c['cwd'], c['name'], 'bash', '-lc',
                         c['env'] + f'python3 tools/vg_check.py -d {args.duration} {topics}'])


def cmd_record(args):
    cfg = load_config(args)
    name = args.name or f"{'_'.join(running_mode_hint())}_{cfg['map']}_" \
                        f"{datetime.datetime.now():%y%m%d_%H%M%S}"
    if not NAME_RE.match(name):
        die('bag name: letters, digits, _ . - only')
    if (ROOT / 'bags' / name).exists():
        die(f'bags/{name} exists')
    if 'record' in running_stages():
        die('a recording is running (./vg.py stop record)')
    cfg['_bag'] = name
    ensure_container('ros2', False)
    if start_proc(stage('record', cfg), False):
        info(f'recording into bags/{name}. Stop: ./vg.py stop record. '
             f'Then: ./vg.py analyze {name}')


def running_mode_hint():
    r = running_stages()
    if 'vslam_to_px4' in r and 'mocap_gt' in r:
        return ['validation']
    if 'mocap_to_px4' in r:
        return ['optitrack']
    if 'localize' in r:
        return ['localization']
    if 'map' in r:
        return ['mapping']
    return ['rec']


def cmd_analyze(args):
    bag = args.bag.rstrip('/').removeprefix('bags/')
    if not (ROOT / 'bags' / bag).is_dir():
        die(f'bags/{bag} not found')
    ensure_container('ros2', False)
    info(f'extracting bags/{bag} -> bags/{bag}.npz (in vanguard_ros2)')
    r = dexec('ros2', CONTAINERS['ros2']['env'] +
              f'python3 compare_cross_rel.py extract bags/{bag} bags/{bag}.npz', capture=False)
    if r.returncode != 0:
        die('extract failed')
    info(f'analyzing -> bags/{bag}.png (on the host)')
    run([sys.executable, str(ROOT / 'compare_cross_rel.py'), 'analyze',
         str(ROOT / 'bags' / f'{bag}.npz'), str(ROOT / 'bags' / f'{bag}.png')] + args.window)


def cmd_maps(args):
    d = MAPPER / 'maps'
    dbs = sorted(d.glob('*.db'), key=lambda p: p.stat().st_mtime)
    if not dbs:
        print('  no maps in mapper/maps')
    for db in dbs:
        n = db.stem
        extra = [x for x, f in (('crosses', f'{n}_crosses.json'), ('pins', f'{n}_pins.json'))
                 if (d / f).exists()]
        t = datetime.datetime.fromtimestamp(db.stat().st_mtime)
        print(f'  {n:<24} {db.stat().st_size / 1e6:7.1f} MB  {t:%Y-%m-%d %H:%M}  '
              f'{" ".join(extra)}')


def cmd_modes(args):
    raw = yaml.safe_load(Path(args.config).read_text())
    for name, m in raw['modes'].items():
        s = f"  {name:<14} {' '.join(m['stages'])}"
        if m.get('set'):
            s += '   (sets ' + ', '.join(f'{k}={v}' for k, v in m['set'].items()) + ')'
        print(s)
    custom = raw.get('custom_stages') or {}
    if custom:
        print('  custom stages: ' + ' '.join(custom))


def cmd_config(args):
    raw = yaml.safe_load(Path(args.config).read_text())
    _, modes = expand([args.mode] if args.mode else [], raw)
    cfg = load_config(args, modes)
    cfg.pop('modes', None)
    print(yaml.safe_dump(cfg, sort_keys=False, default_flow_style=None).rstrip())


def main():
    p = argparse.ArgumentParser(prog='vg', description=__doc__.split('\n\n')[0])
    p.add_argument('-c', '--config', default=str(ROOT / 'vg.yaml'))
    sub = p.add_subparsers(dest='cmd', required=True)

    def with_overrides(sp):
        sp.add_argument('--map')
        sp.add_argument('--set', action='append', metavar='KEY=VALUE')
        return sp

    up = with_overrides(sub.add_parser('up', help='start modes / stages'))
    up.add_argument('targets', nargs='+')
    up.add_argument('-n', '--dry-run', action='store_true', help='print, start nothing')
    up.set_defaults(fn=cmd_up)
    st = sub.add_parser('stop', help='stop stages')
    st.add_argument('stages', nargs='+')
    st.set_defaults(fn=cmd_stop)
    dn = sub.add_parser('down', help='stop all stages')
    dn.add_argument('--containers', action='store_true', help='also stop the mapper container')
    dn.set_defaults(fn=cmd_down)
    sub.add_parser('status', help='what is running').set_defaults(fn=cmd_status)
    lg = sub.add_parser('logs', help='show a stage log')
    lg.add_argument('stage')
    lg.add_argument('-f', '--follow', action='store_true')
    lg.add_argument('-n', type=int, default=40)
    lg.set_defaults(fn=cmd_logs)
    ck = sub.add_parser('check', help='topic rates and PX4 input publishers')
    ck.add_argument('-d', '--duration', type=float, default=3.0)
    ck.set_defaults(fn=cmd_check)
    rc = with_overrides(sub.add_parser('record', help='ros2 bag record the record.topics'))
    rc.add_argument('name', nargs='?')
    rc.set_defaults(fn=cmd_record)
    an = sub.add_parser('analyze', help='compare_cross_rel.py extract + analyze')
    an.add_argument('bag')
    an.add_argument('window', nargs='*', help='t0 t1 [s]')
    an.set_defaults(fn=cmd_analyze)
    sub.add_parser('maps', help='saved maps').set_defaults(fn=cmd_maps)
    sub.add_parser('modes', help='modes and stages').set_defaults(fn=cmd_modes)
    cf = with_overrides(sub.add_parser('config', help='print resolved config'))
    cf.add_argument('mode', nargs='?')
    cf.set_defaults(fn=cmd_config)
    args = p.parse_args()
    args.fn(args)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
