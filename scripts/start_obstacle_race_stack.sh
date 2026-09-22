#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/gouzi/obstacle_race_src}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
RACE_MODE="${RACE_MODE:-1}"
ENABLE_RACE_MODE_SELECTION="${ENABLE_RACE_MODE_SELECTION:-true}"
WAYPOINTS_FILE="${WAYPOINTS_FILE:-${WORKSPACE_DIR}/src/auto_nav_pkg/config/obstacle_waypoints_mirror.yaml}"
BODY_RELATIVE_NAV_CONFIG="${BODY_RELATIVE_NAV_CONFIG:-${WORKSPACE_DIR}/src/auto_nav_pkg/config/orange_pole_body_relative_test_v1.yaml}"
ROS_LOG_DIR="${ROS_LOG_DIR:-${WORKSPACE_DIR}/log/autostart}"

LIVOX_READY_TOPIC="${LIVOX_READY_TOPIC:-/livox/lidar}"
ODOM_READY_TOPIC="${ODOM_READY_TOPIC:-/Odometry}"
SUPERVISOR_STATE_TOPIC="${SUPERVISOR_STATE_TOPIC:-/obstacle_race_supervisor/state}"

LIVOX_READY_TIMEOUT_SEC="${LIVOX_READY_TIMEOUT_SEC:-45}"
ODOM_READY_TIMEOUT_SEC="${ODOM_READY_TIMEOUT_SEC:-60}"
SUPERVISOR_READY_TIMEOUT_SEC="${SUPERVISOR_READY_TIMEOUT_SEC:-45}"

POINT_LIO_RVIZ="${POINT_LIO_RVIZ:-false}"
POINT_LIO_ENABLE_NAV2="${POINT_LIO_ENABLE_NAV2:-true}"
OBSTACLE_SERIAL_DEVICE="${OBSTACLE_SERIAL_DEVICE:-/dev/ttyUSB0}"

PIDS=()

log() {
  printf '[%(%F %T)T] %s\n' -1 "$*"
}

source_ros() {
  mkdir -p "${ROS_LOG_DIR}"
  export ROS_LOG_DIR
  export RCUTILS_LOGGING_BUFFERED_STREAM=1
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  # shellcheck disable=SC1091
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if ((${#PIDS[@]} > 0)); then
    log "stopping child launch processes: ${PIDS[*]}"
    kill "${PIDS[@]}" 2>/dev/null || true
    sleep 2
    kill -TERM "${PIDS[@]}" 2>/dev/null || true
    wait "${PIDS[@]}" 2>/dev/null || true
  fi
  exit "${status}"
}

wait_for_topic_once() {
  local topic="$1"
  local timeout_sec="$2"
  local description="$3"
  local deadline=$((SECONDS + timeout_sec))
  local output_file=/tmp/obstacle_race_wait_topic.log

  log "waiting for ${description}: ${topic} timeout=${timeout_sec}s"
  while ((SECONDS < deadline)); do
    if timeout 3 ros2 topic echo --no-daemon --once "${topic}" \
      --field header.stamp >"${output_file}" 2>&1
    then
      log "${description} ready: ${topic}"
      return 0
    fi
    sleep 1
  done

  log "ERROR: timed out waiting for ${description}: ${topic}"
  log "last ros2 output:"
  sed 's/^/  /' "${output_file}" >&2 || true
  return 1
}

wait_for_supervisor_ready() {
  local timeout_sec="$1"
  local deadline=$((SECONDS + timeout_sec))
  local output_file=/tmp/obstacle_race_wait_supervisor_state.log
  local state

  log "waiting for supervisor state on ${SUPERVISOR_STATE_TOPIC} timeout=${timeout_sec}s"
  while ((SECONDS < deadline)); do
    if timeout 3 ros2 topic echo --no-daemon --once --qos-durability transient_local \
      "${SUPERVISOR_STATE_TOPIC}" >"${output_file}" 2>&1
    then
      state="$(sed -n 's/^[[:space:]]*data:[[:space:]]*//p' "${output_file}" | head -n 1)"
      if [[ -n "${state}" ]]; then
        if [[ "${state}" == "FAILED" ]]; then
          log "ERROR: supervisor reported FAILED"
          sed 's/^/  /' "${output_file}" >&2 || true
          return 1
        fi
        if [[ "${state}" == "WAIT_START" ]]; then
          log "supervisor ready: WAIT_START"
        else
          log "supervisor ready: state=${state} (already past WAIT_START)"
        fi
        return 0
      fi
      log "supervisor state topic exists but no state payload was parsed yet:"
      sed 's/^/  /' "${output_file}" >&2 || true
    fi
    sleep 1
  done

  log "ERROR: timed out waiting for supervisor state"
  sed 's/^/  /' "${output_file}" >&2 || true
  return 1
}

start_launch() {
  local name="$1"
  shift
  log "starting ${name}: $*"
  "$@" &
  PIDS+=("$!")
}

monitor_children() {
  while true; do
    set +e
    wait -n "${PIDS[@]}"
    local status=$?
    set -e
    log "ERROR: one child launch process exited, status=${status}"
    return "${status}"
  done
}

main() {
  trap cleanup EXIT INT TERM
  cd "${WORKSPACE_DIR}"
  source_ros

  log "workspace=${WORKSPACE_DIR}"
  log "race_mode=${RACE_MODE}"
  log "enable_race_mode_selection=${ENABLE_RACE_MODE_SELECTION}"
  log "waypoints=${WAYPOINTS_FILE}"
  log "body_relative_nav_config=${BODY_RELATIVE_NAV_CONFIG}"
  log "ros_log_dir=${ROS_LOG_DIR}"
  log "serial_device=${OBSTACLE_SERIAL_DEVICE}"

  start_launch "livox_mid360" \
    ros2 launch livox_ros_driver2 msg_MID360_launch.py
  wait_for_topic_once "${LIVOX_READY_TOPIC}" "${LIVOX_READY_TIMEOUT_SEC}" "Livox point cloud"

  start_launch "point_lio_mid360" \
    ros2 launch point_lio mapping_mid360.launch.py \
      rviz:="${POINT_LIO_RVIZ}" \
      enable_nav2_bringup:="${POINT_LIO_ENABLE_NAV2}"
  wait_for_topic_once "${ODOM_READY_TOPIC}" "${ODOM_READY_TIMEOUT_SEC}" "Point-LIO odometry"

  start_launch "obstacle_race_visual_supervisor" \
    ros2 launch auto_nav_pkg obstacle_race_visual_supervisor.launch.py \
      race_mode:="${RACE_MODE}" \
      enable_race_mode_selection:="${ENABLE_RACE_MODE_SELECTION}" \
      waypoints_file:="${WAYPOINTS_FILE}" \
      body_relative_nav_config:="${BODY_RELATIVE_NAV_CONFIG}" \
      serial_device:="${OBSTACLE_SERIAL_DEVICE}" \
      enable_point_lio:=false \
      point_lio_enable_nav2:=false
  wait_for_supervisor_ready "${SUPERVISOR_READY_TIMEOUT_SEC}"

  log "stack is ready. Monitoring child launch processes."
  monitor_children
}

main "$@"
