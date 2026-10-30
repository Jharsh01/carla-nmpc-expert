"""Wall-clock time of each stage of a leaderboard tick, one row per tick, to find where a route's time goes.

Stages are timed with `with timing.stage("name"):` anywhere in the process (the evaluator's tick loop, the agent);
times of a stage entered more than once in a tick add up. The evaluator ends each tick with `timing.end_tick(...)`,
which writes the row to the file opened by `timing.open_log` (nothing is written without one):

    {"tick": 12, "t": 0.6, "total": 431.2, "carla.world_tick": 52.0, "agent": 370.1, "agent.simlingo": 301.5, ...}

times in milliseconds. A stage's time includes its sub-stages (named with its name and a dot). Summarised by
scripts/timing_report.py.
"""
import json
import time
from contextlib import contextmanager

_row = {}
_flags = {}
_log = None


@contextmanager
def stage(name):
    t0 = time.perf_counter()
    try:
        yield
    finally:
        _row[name] = _row.get(name, 0.0) + 1e3 * (time.perf_counter() - t0)


def flag(name, value):
    """A value kept with this tick's row (whether SimLingo ran, whether the NMPC planned)."""
    _flags[name] = value


def timed(name):
    """Decorator: time every call of the function as the stage `name`."""
    def decorate(f):
        def timed_f(*args, **kwargs):
            with stage(name):
                return f(*args, **kwargs)
        return timed_f
    return decorate


def wrap(owner, attr, name):
    """Time every call of owner.attr as the stage `name`."""
    setattr(owner, attr, timed(name)(getattr(owner, attr)))


def open_log(path):
    global _log
    close_log()
    _log = open(path, "w")
    _row.clear()  # what ran before the route's first tick (loading it)
    _flags.clear()


def close_log():
    global _log
    if _log is not None:
        _log.close()
        _log = None


def end_tick(**fields):
    """Write this tick's row (with `fields`) and start the next."""
    if _log is not None:
        row = dict(fields, **_flags)
        row.update({k: round(v, 2) for k, v in _row.items()})
        _log.write(json.dumps(row) + "\n")
        _log.flush()
    _row.clear()
    _flags.clear()
