"""Which CPUs each part of a run gets, so that SimLingo's model and the NMPC's solve never take time from the
leaderboard's loop (CARLA's step, the sensors, the scenario tree, the world model) or from each other.

The defaults are for the 8-CPU L4 machine: 4 physical cores, two hardware threads each (CPU n and n + 4 share a
core). CARLA's server and the leaderboard's loop share cores 0 and 1: in synchronous mode each waits for the other,
so they take turns. The NMPC (its candidates solved in parallel) gets core 2, SimLingo's worker core 3. Each part's
set can be changed with CPUS_<PART> (e.g. CPUS_SIMLINGO=3,7); on a machine with other than 8 CPUs nothing is pinned
unless set.
"""
import os

DEFAULTS = {"carla": "0,1,4,5", "main": "0,1,4,5", "nmpc": "2,6", "simlingo": "3,7"}


def cpus(part):
    """The CPUs of `part` (carla, main, nmpc, simlingo), or None to leave it unpinned."""
    value = os.environ.get(f"CPUS_{part.upper()}")
    if value is None and os.cpu_count() == 8:
        value = DEFAULTS[part]
    return {int(c) for c in value.split(",")} if value else None


def pin(part):
    """Pin the calling thread (and the threads it starts from now on) to the CPUs of `part`."""
    mask = cpus(part)
    if mask:
        os.sched_setaffinity(0, mask)  # 0: the calling thread, on Linux
    return mask
