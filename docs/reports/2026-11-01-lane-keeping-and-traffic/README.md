# Lane keeping and traffic, 31 October - 1 November 2026

Summary of the work on lane-band-worker-timing, with `--agent simlingo_nmpc --no-cot --fast` on Town12. The full write-up
is [report.html](report.html), published at https://claude.ai/artifact/NAqPJAdvLePbPfWX9SYWEF. The detail behind each
finding is in [learnings.md](../../learnings.md), and open work is in [todo.md](../../todo.md).

## Result

The car holds the lane's centre and takes only stops from SimLingo, unless SimLingo's line leaves the lane by more than
1 m. Route 0 finished all 15 runs. Routes 1, 8 and 10 end on the 70 s watchdog, waiting in traffic, with no collisions.

| Route | Old path | Lane keeping | How it ended |
| --- | --- | --- | --- |
| 0 ParkingCutIn | 100 % | 100 % | Finished, 15 of 15 runs |
| 1 ParkedObstacle | 18.2 % | 30.3 % | Waits beside the parked car |
| 8 Junction, right turn | 10.3 % | 43.3 % | Waits at a green light |
| 10 Junction, left turn | 35.5 % | 35.5 % | Waits past the stop sign |

Routes 1, 8 and 10 are one run each. On route 0, while moving, lane keeping took steering RMS from 0.060 to 0.019-0.023
rad, heading swing RMS from 1.49 to 0.51-0.63 deg, and 95th percentile lateral acceleration from 1.36 to 0.29-0.44 m/s².

## Main learnings

- The wobble came from forcing the path through the car, not from SimLingo's line. The kink remains while following
  SimLingo through turns and lane changes.
- Manoeuvres can be read from SimLingo against the map lane: 1 m to start, 0.5 m to end. Largest lane-keeping offset 0.57
  m, real manoeuvres 2.6-2.7 m.
- `map.get_waypoint` needs a height. At z = 0 it gave the wrong lane on 19 % of Town12's road points off junctions.
- Repeat runs are mostly the same run: 13 runs gave 3 distinct drives, and the seed made no difference.
- The behaviour layer, not SimLingo, holds the car in traffic. The stream is about 2 s apart and each gate is widened 1 s
  either side. Route 8's stream is merge traffic, but it is classed as crossing traffic.
- Route 8's gaps fit only without margin: 1.9 s available, 2.1-2.8 s needed.
- The last actor field in `nmpc.jsonl` is metres over 5 s, not a speed.

## Decision needed

Route 8's slot leaves about 1 m at each end, and the stream won't brake for the car. The minimum margin and how long to
wait before accepting a smaller one have to be chosen before the merge is built. The first step, the classification fix,
doesn't depend on them.

## Pages

| Copy here | Published, with videos | What it holds |
| --- | --- | --- |
| [lane-keeping.html](lane-keeping.html) | https://claude.ai/artifact/2rxv8KEN2YFgercQNGFiVx | Results, route 0 and 8 videos |
| [driving-behaviours.html](driving-behaviours.html) | https://claude.ai/artifact/TZWK8n2943A3vEPj3HbiMf | Six videos of the behaviours |
| [right-turn-constraints.html](right-turn-constraints.html) | https://claude.ai/artifact/5mgXZSJodoQ7Sk3Cf27chh | Design for traffic, not implemented |
| [route0-wobble.html](route0-wobble.html) | https://claude.ai/artifact/UwjyFoZyNxnzgxAGiZfaHu | Why the old path weaved |
| [route0-stall.html](route0-stall.html) | https://claude.ai/artifact/SBSHPBY6heMSNFJ6YypYzX | The 19.6 % run on the old path |

The published pages are private until shared. The copies here have no videos, as `*.mp4` is ignored by git, so their video
players stay empty; the published pages have them. The stall and wobble pages describe the old path, as of 31 October.
