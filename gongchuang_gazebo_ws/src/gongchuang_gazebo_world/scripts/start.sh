#!/usr/bin/env bash
set -eo pipefail
tool_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$tool_dir/ros_environment.sh"
gc_source_ros
if ! command -v gz >/dev/null; then
  echo 'Gazebo Harmonic is required; install gz-harmonic from the OSRF repository.' >&2
  exit 2
fi
gc_gz_version="$(gz sim --versions 2>/dev/null)"
if [[ ! "$gc_gz_version" =~ (^|[[:space:]])8\. ]]; then
  echo "Gazebo Sim 8 / Harmonic required; found: $gc_gz_version" >&2
  exit 2
fi
pkg="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -d "$pkg/worlds" ]]; then pkg="$(ros2 pkg prefix --share gongchuang_gazebo_world)"; fi
layout="${1:-random_seeded}"
case "$layout" in competition|empty_obstacles|single_obstacle|blocked_corridor|audit_views|audit_stacking|contact_test|batch1_coarse|batch2_ready|batch2_coarse|stacking_demo|random_seeded|random_batch1_coarse|random_batch2_ready|random_batch2_coarse|random_stacking_demo|audit_random|audit_random_stacking|obstacles_0|obstacles_1|obstacles_2|obstacles_3|stress_4|stress_5|stress_6|central_blocked|audit_central_blocked|audit_stress_6|slots_debug|slots_all|central_slots|manual_slots|stress_slots|audit_slots_debug|audit_slots_all|audit_central_slots|audit_manual_slots|audit_stress_slots) ;; *) echo "Unknown layout: $layout" >&2; exit 2;; esac
ws="$(cd "$pkg/../.." && pwd)"
if [[ -f "$ws/install/setup.bash" ]]; then source "$ws/install/setup.bash"; fi
plugin_prefix="$(ros2 pkg prefix gongchuang_gazebo_world)"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$plugin_prefix/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
fieldroot="${GC_FIELD_ROOT:-$pkg}"
export GZ_SIM_RESOURCE_PATH="$fieldroot/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_PARTITION="${GZ_PARTITION:-gongchuang_local}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
if [[ "${GC_SOFTWARE_RENDER:-0}" == 1 ]]; then export LIBGL_ALWAYS_SOFTWARE=1; fi
shift "$(( $# > 0 ? 1 : 0 ))"
exec gz sim -r -v 3 "$fieldroot/worlds/$layout.sdf" "$@"
