#!/usr/bin/env bash
# Run an agent closed loop on Bench2Drive routes, one leaderboard process per route as SimLingo's
# own launcher (start_eval_simlingo.py) does, then compute Bench2Drive's driving score and success rate.
# Run scripts/install_simlingo.sh first (it sets up Bench2Drive too), and for the NMPC build the core
# (cmake -S . -B build && cmake --build build -j).
#
#   scripts/run_bench2drive.sh --agent simlingo_nmpc --routes 0-9   SimLingo through the urban NMPC
#                                                                   (expert/agents/simlingo_nmpc_agent.py),
#                                                                   SimLingo predicting at 1 Hz, the NMPC at 5 Hz
#   scripts/run_bench2drive.sh --agent simlingo ...                 SimLingo with its PID, as published (the default)
#
#   scripts/run_bench2drive.sh                       all 220 routes (days on one GPU: see below)
#   scripts/run_bench2drive.sh --routes 0-19         routes 0 to 19 of bench2drive_split/
#   scripts/run_bench2drive.sh --routes 3,17,120     these routes
#   scripts/run_bench2drive.sh --seed 2 --tries 1    traffic manager seed (default 1), attempts per route (default 2)
#   scripts/run_bench2drive.sh --debug               record each route: front camera with the planned path, 4 frames/s
#   scripts/run_bench2drive.sh --video               record each route from two cameras on the car (a chase view and
#                                                    one from above, not seen by the agent) into
#                                                    videos/<route>_<scenario>.mp4, real time, with the NMPC's log
#                                                    under them (scripts/make_video.py)
#   scripts/run_bench2drive.sh --no-cot              no commentary before the waypoints (about 3x faster)
#   scripts/run_bench2drive.sh --agent simlingo_nmpc --live
#                                                    watch the NMPC's solves live in a browser (expert/live_view.py):
#                                                    http://localhost:8765 on this machine (NMPC_LIVE_PORT to
#                                                    change it); from elsewhere, ssh -L 8765:localhost:8765 first
#   scripts/run_bench2drive.sh --max-stopped 30 --max-time 70
#                                                    the watchdog (these are the defaults; 0 turns either off): a
#                                                    route is stopped and recorded as crashed once the ego has been
#                                                    at rest for 30 s, or after 70 s, in simulation time
#                                                    (scripts/b2d_evaluator.py). Its score so far still counts
#                                                    The evaluator also checks the simulation is synchronous with a
#                                                    fixed 0.05 s step, and prints how fast it runs against real time
#                                                    ("Sync:" lines in out/<route>.log, summarised after each route)
#   scripts/run_bench2drive.sh --fast                eval() and LoRA merged into the LLM (see scripts/simlingo.patch)
#   scripts/run_bench2drive.sh --out results/bench2drive/latest
#                                                    results into this folder in place of the default one below
#
# By default the agent runs as SimLingo's own evaluation does: commentary generated every step
# (use_cot), and the model left in training mode, so its LoRA dropout is on. Measured on an L4 per 50 ms
# step: about 3 s by default and 0.9 s with --no-cot (both with a second CARLA server running), 0.4 s with
# --no-cot --fast on an otherwise idle GPU, which is about a day for all 220 routes.
# Every route starts its own CARLA server (port 20000 up, so a server already on 2000 is left alone)
# and waits 60 s for it. Routes with a finished result are skipped, so a stopped run carries on.
# Results: results/bench2drive/<agent>[-nocot][-fast]/seed<N>/res/<route>.json (one per route),
# res/merged.json (driving score, success rate), out/<route>.log (leaderboard output),
# viz/<route>/ (per-step metric_info.json). --debug runs go to their own folder (<mode>-video), with
# videos/<route>_<scenario>.mp4: red dots the planned path, green the speed waypoints (0.25 s apart), blue the target point
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/env.sh

routes=all seed=1 tries=2 debug=0 cot=1 fast=0 agent=simlingo max_stopped=30 max_time=70 live=0 out= record=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --routes) routes=$2; shift ;;
        --agent) agent=$2; shift ;;
        --seed) seed=$2; shift ;;
        --tries) tries=$2; shift ;;
        --debug) debug=1 ;;
        --video) record=1 ;;
        --no-cot) cot=0 ;;
        --fast) fast=1 ;;
        --max-stopped) max_stopped=$2; shift ;;
        --max-time) max_time=$2; shift ;;
        --live) live=1 ;;
        --out) out=$2; shift ;;
        -h|--help) awk 'NR > 1 && !/^#/ {exit} NR > 1 {sub(/^# ?/, ""); print}' "$0"; exit 0 ;;
        *) echo "unknown option $1 (-h lists them)" >&2; exit 1 ;;
    esac
    shift
done

SIMLINGO=$EXTERNAL/simlingo
VPY=$EXTERNAL/simlingo_venv/bin/python
CKPT="$EXTERNAL/simlingo_ckpt/simlingo/checkpoints/epoch=013.ckpt/pytorch_model.pt"
SPLIT=$SIMLINGO/leaderboard/data/bench2drive_split
[[ -x $VPY && -f $CARLA_ROOT/.carla-additional-maps ]] \
    || { echo "SimLingo and Bench2Drive are not set up: run scripts/install_simlingo.sh" >&2; exit 1; }
[[ -f $CKPT ]] || { echo "no SimLingo checkpoint: run scripts/install_simlingo.sh" >&2; exit 1; }
AGENT_CONFIG=$CKPT
case $agent in
    simlingo) AGENT_FILE=team_code/agent_simlingo.py ;;
    simlingo_nmpc)
        ls "$REPO"/build/python/expertctl*.so >/dev/null 2>&1 || { echo "build the core first (cmake --build build)" >&2; exit 1; }
        AGENT_FILE=$REPO/expert/agents/simlingo_nmpc_agent.py ;;
    *) echo "unknown agent $agent (simlingo or simlingo_nmpc)" >&2; exit 1 ;;
esac
tag=$agent
[[ $cot == 0 ]] && tag+=-nocot
[[ $fast == 1 ]] && tag+=-fast
[[ $debug == 1 ]] && tag+=-video
out_root=$REPO/results/bench2drive/$tag/seed$seed
[[ -n $out ]] && out_root=$(realpath -m "$out")
mkdir -p "$out_root"/{res,out,viz}

ids=()
if [[ $routes == all ]]; then
    ids=($(seq 0 219))
else
    IFS=, read -ra parts <<< "$routes"
    for p in "${parts[@]}"; do
        if [[ $p == *-* ]]; then ids+=($(seq "${p%-*}" "${p#*-}")); else ids+=("$p"); fi
    done
fi

# the agent and the leaderboard are imported from the SimLingo tree; weather.xml and the
# InternVL2 cache (pretrained/) are looked up relative to the working directory. SimLingo's tree
# goes first: Bench2Drive's leaderboard has a team_code package of its own that would shadow it
export CARLA_ROOT WORK_DIR=$SIMLINGO
export SCENARIO_RUNNER_ROOT=$SIMLINGO/Bench2Drive/scenario_runner
export PYTHONPATH="$SIMLINGO:$SIMLINGO/team_code:$CARLA_ROOT/PythonAPI/carla:$SIMLINGO/Bench2Drive/leaderboard:$SCENARIO_RUNNER_ROOT:$REPO/build/python:$REPO"
export SIMLINGO_DEBUG=$debug SIMLINGO_COT=$cot SIMLINGO_FAST=$fast
export B2D_MAX_STOPPED=$max_stopped B2D_MAX_TIME=$max_time  # the watchdog, in scripts/b2d_evaluator.py
if [[ $record == 1 ]]; then export B2D_VIDEO=1; else unset B2D_VIDEO; fi  # the cameras, in scripts/b2d_evaluator.py
if [[ $live == 1 ]]; then
    export NMPC_LIVE_PORT=${NMPC_LIVE_PORT:-8765}
    echo "live view: http://localhost:$NMPC_LIVE_PORT (from elsewhere: ssh -L $NMPC_LIVE_PORT:localhost:$NMPC_LIVE_PORT <this machine>)"
else
    unset NMPC_LIVE_PORT
fi
FFMPEG=$("$PY" -c 'import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())')  
cd "$SIMLINGO"

finished() {  # the result file exists, the route ran to its end and the run did not fail (a watchdog stop is a result)
    "$VPY" - "$1" <<'EOF'
import json, sys
try:
    cp = json.load(open(sys.argv[1]))["_checkpoint"]
except Exception:
    sys.exit(1)
p = cp["progress"]
ok = len(p) == 2 and p[0] >= p[1] and all(not r["status"].startswith("Failed") or "watchdog" in r["status"]
                                          for r in cp["records"])
sys.exit(0 if ok else 1)
EOF
}

for i in "${ids[@]}"; do
    name=$(printf "bench2drive_%02d" "$i")
    route=$SPLIT/$name.xml
    [[ -f $route ]] || { echo "no route $route" >&2; exit 1; }
    res=$out_root/res/$name.json
    for ((t = 1; t <= tries; t++)); do
        finished "$res" && break
        echo "== $name (attempt $t/$tries)  $(date +%H:%M:%S)"
        rm -f "$res"
        SAVE_PATH=$out_root/viz/ ROUTES=$route timeout 3600 "$VPY" -u \
            "$REPO/scripts/b2d_evaluator.py" \
            --routes="$route" --repetitions=1 --track=SENSORS --checkpoint="$res" --timeout=600 \
            --agent="$AGENT_FILE" --agent-config="$AGENT_CONFIG" \
            --traffic-manager-seed="$seed" --port=20000 --traffic-manager-port=30000 \
            > "$out_root/out/$name.log" 2>&1 || true
        # the leaderboard leaves its server behind when it is killed; only ours have -graphicsadapter
        pkill -9 -f 'CarlaUE4.*-graphicsadapter' 2>/dev/null || true
        sleep 2
    done
    grep -a "Sync:" "$out_root/out/$name.log" | sed -n '1p;$p' | sed 's/^/   /' || true
    if finished "$res"; then
        "$VPY" - "$res" <<'EOF'
import json, sys
r = json.load(open(sys.argv[1]))["_checkpoint"]["records"][0]
s = r["scores"]
bad = [k for k, v in r["infractions"].items() if v and k != "min_speed_infractions"]
print(f"   {r['status']}: score {s['score_composed']:.1f}, route {s['score_route']:.1f}%, penalty {s['score_penalty']:.2f}"
      + (f", {', '.join(bad)}" if bad else ""))
EOF
    else
        echo "   no result after $tries attempts: see $out_root/out/$name.log"
    fi
    if [[ $record == 1 && -d $out_root/viz/$name/video ]]; then
        mkdir -p "$out_root/videos"
        scenario=$("$VPY" -c "import sys, xml.etree.ElementTree as ET; print(next(ET.parse(sys.argv[1]).iter('scenario')).get('type'))" "$route")
        "$VPY" "$REPO/scripts/make_video.py" "$out_root/viz/$name" "$out_root/videos/${name#bench2drive_}_$scenario.mp4" \
            --title "$name  $agent" | sed 's/^/   video: /' && rm -rf "$out_root/viz/$name/video"
    fi
    if [[ $debug == 1 ]]; then  # the agent's frames (every 5th step) into one video, real time
        frames=$(find "$out_root/viz" -type d -path "*/${name}_*/images" | sort | tail -1)
        if [[ -n $frames ]] && ls "$frames"/*.png >/dev/null 2>&1; then
            mkdir -p "$out_root/videos"
            scenario=$("$VPY" -c "import sys, xml.etree.ElementTree as ET; print(next(ET.parse(sys.argv[1]).iter('scenario')).get('type'))" "$route")
            video=$out_root/videos/${name#bench2drive_}_$scenario.mp4
            ls "$frames" | grep '\.png$' | sort -n | sed "s|^|file '$frames/|; s|\$|'|" > "$frames/list.txt"
            "$FFMPEG" -loglevel error -y -f concat -safe 0 -r 4 -i "$frames/list.txt" \
                -vf "crop=iw:ih-312:0:0,fps=4" -c:v libx264 -pix_fmt yuv420p -crf 23 "$video" \
                && rm -f "$frames"/*.png "$frames/list.txt" && echo "   video: $video"
        fi
    fi
done

ls "$out_root"/res/bench2drive_*.json >/dev/null 2>&1 || { echo "no route has a result yet"; exit 1; }
"$VPY" Bench2Drive/tools/merge_route_json.py -f "$out_root/res" > /dev/null
"$VPY" - "$out_root/res/merged.json" <<'EOF'
import json, sys
m = json.load(open(sys.argv[1]))
print(f"\n{m['eval num']} routes: driving score {m['driving score']:.2f}, success rate {100 * m['success rate']:.1f} %"
      + ("" if m["eval num"] == 220 else "  (Bench2Drive's numbers are over all 220 routes)"))
print(f"merged results: {sys.argv[1]}")
EOF
