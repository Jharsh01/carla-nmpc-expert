#!/usr/bin/env bash
# Run PDM-Lite on Bench2Drive's routes, several CARLA servers at once, then score the lot.
#
#   scripts/run_pdm_lite_all.sh --out ~/b2d_results/pdm_lite                  all 220 routes, 3 at a time
#   scripts/run_pdm_lite_all.sh --out DIR --routes 0-43 --jobs 2 --no-video   some routes
#
# Each route goes through scripts/run_pdm_lite_route.sh (one folder per route, with its video and map). Each job
# takes the next route nobody has started, so a long route holds up only its own job. Routes already done are
# skipped, so a stopped run carries on where it was. A route that fails is left as it is (one attempt; --tries N
# for more). At the end scripts/summarize_b2d.py writes <out>/summary.md, routes.csv and res/merged.json
# (Bench2Drive's driving score and success rate). Progress: <out>/run.log.
set -euo pipefail
cd "$(dirname "$0")/.."

out= routes=all jobs=3 extra=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --out) out=$2; shift ;;
        --routes) routes=$2; shift ;;
        --jobs) jobs=$2; shift ;;
        --seed) extra+=(--seed "$2"); shift ;;
        --tries) extra+=(--tries "$2"); shift ;;
        --no-video) extra+=(--no-video) ;;
        -h|--help) awk 'NR > 1 && !/^#/ {exit} NR > 1 {sub(/^# ?/, ""); print}' "$0"; exit 0 ;;
        *) echo "unknown option $1 (-h lists them)" >&2; exit 1 ;;
    esac
    shift
done
[[ -n $out ]] || { echo "--out is needed (-h)" >&2; exit 1; }
out=$(realpath -m "$out")
mkdir -p "$out"

ids=()
if [[ $routes == all ]]; then
    ids=($(seq 0 219))
else
    IFS=, read -ra parts <<< "$routes"
    for p in "${parts[@]}"; do
        if [[ $p == *-* ]]; then ids+=($(seq "${p%-*}" "${p#*-}")); else ids+=("$p"); fi
    done
fi

claims=$out/.claims
rm -rf "$claims" && mkdir -p "$claims"
worker() {
    local job=$1
    for i in "${ids[@]}"; do
        mkdir "$claims/$i" 2>/dev/null || continue  # taken by another job
        scripts/run_pdm_lite_route.sh --route "$i" --out "$out" --job "$job" "${extra[@]}" || true
    done
}

echo "${#ids[@]} routes, $jobs jobs, from $(date '+%F %T')" | tee -a "$out/run.log"
for ((k = 0; k < jobs; k++)); do
    worker "$k" >> "$out/run.log" 2>&1 &
    sleep 20  # the servers start one after another, not all loading a town at once
done
wait
rm -rf "$claims"
echo "finished $(date '+%F %T')" | tee -a "$out/run.log"
source scripts/env.sh
"$PY" scripts/summarize_b2d.py "$out" | tee -a "$out/run.log"
