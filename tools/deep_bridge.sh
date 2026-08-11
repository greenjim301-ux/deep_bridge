#!/usr/bin/env bash
# deep_bridge start/stop script
#
# Usage: deep_bridge.sh {start|stop|restart|status|log} [extra roslaunch args...]
#
# - start prepares the environment automatically: if roslaunch is not on
#   PATH and mamba is available, activates ros_host, then sources the
#   workspace's devel/setup.bash (workspace defaults to three levels above
#   this script's location; override with the ROS_WS env var if the path
#   differs on a given machine).
# - If the ROS master is unreachable, a roscore is started in the
#   background automatically; its pid is recorded in <ws>/run/roscore.pid.
#   roscore is shared across package scripts, so stop does not kill it;
#   kill $(cat run/roscore.pid) manually when it's no longer needed.
# - stop sends SIGINT first so roslaunch can shut down cleanly; if it has
#   not exited after 15s, escalates to SIGTERM/SIGKILL.
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_DIR=${ROS_WS:-$(cd "$SCRIPT_DIR/../../.." && pwd)}

NAME=deep_bridge
LAUNCH_PKG=deep_bridge
LAUNCH_FILE=deep_bridge.launch
DEFAULT_ARGS=()

RUN_DIR=$WS_DIR/run
PID_FILE=$RUN_DIR/$NAME.pid
LOG_FILE=$RUN_DIR/$NAME.log
ROSCORE_PID_FILE=$RUN_DIR/roscore.pid
ROSCORE_LOG_FILE=$RUN_DIR/roscore.log

pid_alive() { [ -f "$1" ] && kill -0 "$(cat "$1")" 2>/dev/null; }

setup_env() {
    if ! command -v roslaunch >/dev/null 2>&1 && command -v mamba >/dev/null 2>&1; then
        # dev machine: ROS lives in the mamba ros_host env; on-device ROS
        # installs are on PATH already, so this branch is skipped there
        eval "$(mamba shell hook --shell bash)"
        mamba activate ros_host
    fi
    if [ ! -f "$WS_DIR/devel/setup.bash" ]; then
        echo "Could not find $WS_DIR/devel/setup.bash - run catkin_make first, or set ROS_WS to point at the workspace" >&2
        exit 1
    fi
    source "$WS_DIR/devel/setup.bash"
}

# roslaunch's entire output is redirected into $LOG_FILE, so ROS_INFO/printf
# from the node land in the file rather than the terminal. Once startup
# finishes, grep the final-status line back out of the log and echo it so
# you don't have to open the log file every time. The wait can be tuned
# with the FINAL_STATUS_WAIT env var (defaults wide because the node waits
# stand_settle_sec for the stand-up request plus mode_settle_sec for any
# gait switch, on top of status_wait_timeout_sec for each readback, before
# that line is printed).
FINAL_STATUS_MARK='Final status'
FINAL_STATUS_WAIT=${FINAL_STATUS_WAIT:-20}

report_final_status() {
    local line=""
    for _ in $(seq 1 "$FINAL_STATUS_WAIT"); do
        pid_alive "$PID_FILE" || break
        line=$(grep -a -m1 "$FINAL_STATUS_MARK" "$LOG_FILE" 2>/dev/null || true)
        [ -n "$line" ] && break
        sleep 1
    done

    if [ -n "$line" ]; then
        echo "$line"
    else
        echo "Warning: did not see the final status in the log within ${FINAL_STATUS_WAIT}s, check $LOG_FILE" >&2
    fi
}

ensure_master() {
    if timeout 3 rostopic list >/dev/null 2>&1; then return; fi
    if pid_alive "$ROSCORE_PID_FILE"; then return; fi # roscore just started, not ready yet
    echo "ROS master unreachable, starting roscore in the background (log: $ROSCORE_LOG_FILE)"
    nohup roscore >"$ROSCORE_LOG_FILE" 2>&1 &
    echo $! >"$ROSCORE_PID_FILE"
}

start() {
    if pid_alive "$PID_FILE"; then
        echo "$NAME is already running (pid $(cat "$PID_FILE"))"
        return
    fi
    setup_env
    mkdir -p "$RUN_DIR"
    ensure_master
    nohup roslaunch --wait "$LAUNCH_PKG" "$LAUNCH_FILE" "${DEFAULT_ARGS[@]}" "$@" >"$LOG_FILE" 2>&1 &
    echo $! >"$PID_FILE"
    sleep 3
    if pid_alive "$PID_FILE"; then
        echo "$NAME started (pid $(cat "$PID_FILE")), log: $LOG_FILE"
        report_final_status
    else
        echo "$NAME failed to start, log tail:" >&2
        tail -n 20 "$LOG_FILE" >&2
        rm -f "$PID_FILE"
        exit 1
    fi
}

stop() {
    if ! pid_alive "$PID_FILE"; then
        echo "$NAME is not running"
        rm -f "$PID_FILE"
        return
    fi
    local pid
    pid=$(cat "$PID_FILE")
    echo "Stopping $NAME (pid $pid) ..."
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 15); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 1
    done
    if kill -0 "$pid" 2>/dev/null; then
        echo "Did not exit within 15s of SIGINT, escalating to SIGTERM/SIGKILL"
        kill -TERM "$pid" 2>/dev/null || true
        sleep 3
        kill -KILL "$pid" 2>/dev/null || true
    fi
    rm -f "$PID_FILE"
    echo "$NAME stopped"
}

status() {
    if pid_alive "$PID_FILE"; then
        echo "$NAME running (pid $(cat "$PID_FILE"))"
    else
        echo "$NAME not running"
    fi
}

cmd=${1:-}
shift 2>/dev/null || true
case "$cmd" in
    start) start "$@" ;;
    stop) stop ;;
    restart)
        stop
        start "$@"
        ;;
    status) status ;;
    log) exec tail -n 50 -f "$LOG_FILE" ;;
    *)
        echo "Usage: $(basename "$0") {start|stop|restart|status|log} [extra roslaunch args...]" >&2
        exit 1
        ;;
esac
