#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
tool_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg="$(cd "$tool_dir/.." && pwd)"
ws="${GC_WORKSPACE:-$HOME/gongchuang_gazebo_ws}"
seed="${1:-auto}"; count="${2:-auto}"; stage="${3:-random_seeded}"
if [[ "$seed" == auto ]]; then seed="$(python3 -c 'import secrets; print(secrets.randbits(32))')"; fi
[[ "$seed" =~ ^[0-9]+$ ]] || { echo "Seed must be uint32" >&2; exit 2; }
case "$stage" in random_seeded|random_batch1_coarse|random_batch2_ready|random_batch2_coarse|random_stacking_demo) ;; *) echo "Invalid stage" >&2; exit 2;; esac
shift "$(( $# >= 3 ? 3 : $# ))"
generation_args=(--seed "$seed")
if [[ "$count" != auto ]]; then generation_args+=(--obstacle-count "$count"); fi
# Keep argv as an array; modes and IDs also identify the isolated output and partition.
suffix="$(python3 -c 'import hashlib,sys; print(hashlib.sha256(repr(sys.argv[1:]).encode()).hexdigest()[:12])' "$count" "$@")"
export GC_FIELD_ROOT="$ws/generated/seed_${seed}_slots_${suffix}"
export PYTHONPATH="$ws/.python_deps${PYTHONPATH:+:$PYTHONPATH}"
export GZ_PARTITION="${GZ_PARTITION:-gongchuang_seed_${seed}_slots_${suffix}}"
if gz service -l | grep -q "^/world/gongchuang/control$"; then
  echo "This selection is already running in $GZ_PARTITION; close its Gazebo window first." >&2; exit 2
fi
echo "Scene seed=$seed stage=$stage bundle=$GC_FIELD_ROOT"
python3 "$tool_dir/generate.py" "${generation_args[@]}" "$@" --output "$GC_FIELD_ROOT"
exec bash "$tool_dir/start.sh" "$stage"
