#!/usr/bin/env python3
"""
Trial records - the attribute engine's memory. Added Nov 9, 2026.

Every finished trial saves one row to the zeroGravity R&D database
(~/zeroGravity-rnd/lab/database/zerogravity_rnd.db, table attribute_trials,
the same database telemetry_bridge.py logs simulation_runs into), linked to
the simulation run that was live at the time: when, which trial, the raw measurements (metres, seconds, %...), the
scores they earn under config/attributes.json, a snapshot of the
ArduPilot tuning parameters at the time, and the loadout (battery, props,
payload) from attributes.json. That snapshot is what turns perks into
measured cause and effect: change the tune or the battery, run the trials
again, and the history shows what moved.

The scoring rule lives here AND is mirrored by the Attributes room
(scoreOf in dashboard.html) - keep them identical:
    score = 100 * (value - worst) / (best - worst), clamped to 0..100
"""

import json
import sqlite3
import time
from pathlib import Path

# Sim trials belong with the other simulation data in the R&D database
# (real-drone trials, later, go in the Shadow database, never here).
DB_PATH = Path.home() / 'zeroGravity-rnd' / 'lab' / 'database' / 'zerogravity_rnd.db'
CONFIG_PATH = Path.home() / 'shadow_ws' / 'src' / 'shadow_drone' / 'config' / 'attributes.json'

# ArduPilot parameters worth keeping with every result (tuning + limits).
PARAM_SNAPSHOT = [
    'ATC_RAT_RLL_P', 'ATC_RAT_RLL_I', 'ATC_RAT_RLL_D',
    'ATC_RAT_PIT_P', 'ATC_RAT_PIT_I', 'ATC_RAT_PIT_D',
    'ATC_RAT_YAW_P', 'ATC_RAT_YAW_I',
    'ATC_ANG_RLL_P', 'ATC_ANG_PIT_P', 'ATC_ANG_YAW_P',
    'PSC_POSXY_P', 'PSC_VELXY_P', 'PSC_POSZ_P', 'PSC_VELZ_P', 'PSC_ACCZ_P',
    'WP_SPD', 'WP_ACC', 'MOT_THST_HOVER', 'MOT_THST_EXPO', 'MOT_SPIN_MIN', 'MOT_SPIN_MAX',
    'ATC_RATE_Y_MAX', 'BATT_CAPACITY',
]


def load_config():
    try:
        return json.loads(CONFIG_PATH.read_text())
    except (OSError, ValueError):
        return {'attributes': {}, 'loadout': {}}


def score(value, best, worst):
    if value is None or best == worst:
        return None
    return round(max(0.0, min(1.0, (value - worst) / (best - worst))) * 100)


def score_metrics(metrics, config=None):
    """{metric_key: value} -> {metric_key: {attribute, label, value, score}}"""
    config = config or load_config()
    out = {}
    for attr, defs in config.get('attributes', {}).items():
        for key, d in defs.items():
            if key in metrics and metrics[key] is not None:
                out[key] = {'attribute': attr, 'label': d['label'], 'value': metrics[key],
                            'score': score(metrics[key], d['best'], d['worst'])}
    return out


def read_params(body, names=PARAM_SNAPSHOT, timeout_s=2.0):
    """Best effort: FCU parameters as MAVROS 2 exposes them (ROS parameters
    on the /mavros/param node). Missing or unreachable = empty dict."""
    try:
        import rclpy
        from rcl_interfaces.srv import GetParameters
    except ImportError:
        return {}
    cli = body.create_client(GetParameters, '/mavros/param/get_parameters')
    if not cli.wait_for_service(timeout_sec=timeout_s):
        return {}
    req = GetParameters.Request()
    req.names = list(names)
    fut = cli.call_async(req)
    deadline = time.time() + timeout_s
    while not fut.done() and time.time() < deadline:
        rclpy.spin_once(body, timeout_sec=0.05)
    if not fut.done() or fut.result() is None:
        return {}
    out = {}
    for name, v in zip(names, fut.result().values):
        # ParameterValue: type 2 = integer, 3 = double, 0 = not set
        if v.type == 3:
            out[name] = round(v.double_value, 6)
        elif v.type == 2:
            out[name] = v.integer_value
    return out


def _db():
    if not DB_PATH.parent.exists():
        raise FileNotFoundError(f'R&D database folder missing: {DB_PATH.parent}')
    # timeout: telemetry_bridge.py writes to this file every second
    conn = sqlite3.connect(str(DB_PATH), timeout=5.0)
    conn.execute('''CREATE TABLE IF NOT EXISTS attribute_trials (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        simulation_run_id INTEGER REFERENCES simulation_runs(id),
        created_at TEXT NOT NULL,
        t REAL NOT NULL,
        trial TEXT NOT NULL,
        metrics TEXT NOT NULL,
        scores TEXT NOT NULL,
        params TEXT NOT NULL,
        loadout TEXT NOT NULL,
        notes TEXT)''')
    return conn


def _current_run_id(conn):
    """The simulation_runs row telemetry_bridge.py has open, if any."""
    try:
        row = conn.execute(
            "SELECT id FROM simulation_runs WHERE status = 'running' ORDER BY id DESC LIMIT 1").fetchone()
    except sqlite3.Error:
        return None
    return row[0] if row else None


def save_trial(trial, metrics, params=None, notes=None):
    from datetime import datetime, timezone
    config = load_config()
    scored = score_metrics(metrics, config)
    conn = _db()
    with conn:
        conn.execute(
            'INSERT INTO attribute_trials (simulation_run_id, created_at, t, trial, metrics, scores, params, loadout, notes) '
            'VALUES (?,?,?,?,?,?,?,?,?)',
            (_current_run_id(conn), datetime.now(timezone.utc).isoformat(), time.time(), trial,
             json.dumps(metrics), json.dumps(scored), json.dumps(params or {}),
             json.dumps(config.get('loadout', {})), notes))
    conn.close()
    return scored
