#!/usr/bin/env bash
# Run line N of routes.txt (1-based) with video, at most 10 min, into runs/<route>_<Type>/; copy the video to videos/.
set -u
cd "$(dirname "$0")/.."
read r t < <(sed -n "${1}p" scenario_sweep/routes.txt)
name=$(printf "%03d_%s" "$r" "$t")
out=scenario_sweep/runs/$name
echo "[$1/39] route $r $t  start $(date +%T)"
timeout 600 scripts/run_bench2drive.sh --agent simlingo_nmpc --routes "$r" --tries 1 --no-cot --fast --video --out "$out" > "$out.log" 2>&1
echo "exit $?"
grep -E "Completed|Failed|video:" "$out.log" | sed 's/\x1b\[[0-9;]*m//g'
v=$(ls "$out"/videos/*.mp4 2>/dev/null | head -1); [ -n "$v" ] && cp "$v" "scenario_sweep/videos/$name.mp4" && echo "video -> scenario_sweep/videos/$name.mp4"
pkill -9 -f "carla-rpc-port=2000[0-9]" 2>/dev/null; pkill -9 -f "leaderboard_evaluator" 2>/dev/null; true
echo "end $(date +%T)"
