#!/usr/bin/env bash
# ubuntu-22.04 defaults to Humble; an already selected ROS environment is respected.
gc_source_ros() {
  local gc_distro="${GC_ROS_DISTRO:-${ROS_DISTRO:-humble}}"
  local gc_setup="${GC_ROS_SETUP:-/opt/ros/$gc_distro/setup.bash}"
  if [[ ! -f "$gc_setup" ]]; then
    echo "ROS setup not found: $gc_setup. Install ROS 2 Humble for Ubuntu 22.04, or set GC_ROS_DISTRO / GC_ROS_SETUP." >&2
    return 2
  fi
  source "$gc_setup"
}
