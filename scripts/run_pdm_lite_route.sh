#!/usr/bin/env bash
# Run PDM-Lite (expert/agents/pdm_lite_agent.py) on one Bench2Drive route and write a folder with its log, a video
# and a route map. scripts/run_pdm_lite_all.sh runs every route this way, several at once.
#
#   scripts/run_pdm_lite_route.sh --route 2 --out ~/b2d_results/pdm_lite
#   scripts/run_pdm_lite_route.sh --route 2 --out DIR --no-video --seed 1 --job 1 --tries 1
#
# Results: <out>/<NN>_<scenario>/ with summary.md, map.png, video.mp4, steps.csv (each tick), result.json (the
# leaderboard's) and leaderboard.log. A route whose folder already has a summary is skipped. The leaderboard starts
# its own CARLA server in ports 20000 + 100 job and up (traffic manager 30000 + 100 job; scripts/b2d_evaluator.py),
# and only the servers in that range are stopped, so jobs with different numbers can run side by side.
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/env.sh

route= out= seed=1 video=1 job=0 tries=1
while [[ $# -gt 0 ]]; do
    case "$1" in
        --route) route=$2; shift ;;
        --out) out=$2; shift ;;
        --seed) seed=$2; shift ;;
        --job) job=$2; shift ;;
        --tries) tries=$2; shift ;;
        --no-video) video=0 ;;
        -h|--help) awk 'NR > 1 && !/^#/ {exit} NR > 1 {sub(/^# ?/, ""); print}' "$0"; exit 0 ;;
        *) echo "unknown option $1 (-h lists them)" >&2; exit 1 ;;
    esac
    shift
done
[[ -n $route && -n $out ]] || { echo "--route and --out are needed (-h)" >&2; exit 1; }

SIMLINGO=$EXTERNAL/simlingo
VPY=$EXTERNAL/simlingo_venv/bin/python
name=$(printf "bench2drive_%02d" "$route")
xml=$SIMLINGO/leaderboard/data/bench2drive_split/$name.xml
[[ -f $xml ]] || { echo "no route $xml" >&2; exit 1; }
scenario=$("$VPY" -c "import sys, xml.etree.ElementTree as ET; s = next(ET.parse(sys.argv[1]).iter('scenario'), None); print(s.get('type') if s is not None else 'none')" "$xml")
run=$(realpath -m "$out")/$(printf "%02d" "$route")_$scenario
[[ -f $run/summary.md ]] && { echo "== $name $scenario: done already"; exit 0; }
port=$((20000 + 100 * job)) tm_port=$((30000 + 100 * job))

# team_code/ for the autopilot, the repository for the wrapper; SimLingo's tree first (see run_bench2drive.sh)
export CARLA_ROOT WORK_DIR=$SIMLINGO
export SCENARIO_RUNNER_ROOT=$SIMLINGO/Bench2Drive/scenario_runner
export PYTHONPATH="$SIMLINGO:$SIMLINGO/team_code:$CARLA_ROOT/PythonAPI/carla:$SIMLINGO/Bench2Drive/leaderboard:$SCENARIO_RUNNER_ROOT:$REPO"
export PDM_OUT=$run PDM_VIDEO=$video
cd "$SIMLINGO"

finished() {  # the leaderboard's result exists and the route ran to its end: a driving failure (blocked, timed
    # out, off the route) is a result; the simulator or the agent crashing, or the agent not starting, is not
    "$VPY" - "$run/result.json" <<'EOF'
import json, sys
try:
    cp = json.load(open(sys.argv[1]))["_checkpoint"]
except Exception:
    sys.exit(1)
p = cp["progress"]
broken = ("Simulation crashed", "Agent crashed", "Agent couldn't be set up")
ok = len(p) == 2 and p[0] >= p[1] and cp["records"] and not any(b in r["status"] for r in cp["records"] for b in broken)
sys.exit(0 if ok else 1)
EOF
}

kill_servers() {  # every CARLA server in this job's ports (the evaluator moves on from a busy port)
    ps -eo pid,args | awk -v lo="$port" -v hi=$((port + 99)) 'match($0, /-carla-rpc-port=[0-9]+/) {
        p = substr($0, RSTART + 16, RLENGTH - 16) + 0; if (p >= lo && p <= hi) print $1 }' | xargs -r kill -9 2>/dev/null || true
}

for ((t = 1; t <= tries; t++)); do
    echo "== $name $scenario, PDM-Lite, job $job (attempt $t/$tries)  $(date +%H:%M:%S)  ->  $run"
    kill_servers
    rm -rf "$run" && mkdir -p "$run"
    SAVE_PATH=$run/viz/ ROUTES=$xml timeout 1800 "$VPY" -u "$REPO/scripts/b2d_evaluator.py" \
        --routes="$xml" --repetitions=1 --track=MAP --checkpoint="$run/result.json" --timeout=600 \
        --agent="$REPO/expert/agents/pdm_lite_agent.py" --agent-config=none \
        --traffic-manager-seed="$seed" --port="$port" --traffic-manager-port="$tm_port" \
        > "$run/leaderboard.log" 2>&1 || true
    kill_servers  # the leaderboard can leave its server behind
    sleep 5
    rm -rf "$run/viz"
    finished && [[ -f $run/steps.csv ]] && break
done

finished && [[ -f $run/steps.csv ]] || { echo "   no result after $tries attempts: see $run/leaderboard.log" >&2; exit 1; }
"$VPY" "$REPO/scripts/route_report.py" "$run" "$xml" | sed -n '3,5p'
