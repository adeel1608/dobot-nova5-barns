#!/usr/bin/env bash
#
# entrypoint.sh  –  robot container bootstrap (simplified)
# ───────────────────────────────────────────────────────────────
set -euo pipefail      # safe-by-default shell

###############################################################################
# 0.  Workspace & helper
###############################################################################
WS=/root/ros_ws
log() { printf "\033[1;35m[entrypoint]\033[0m %s\n" "$*"; }

###############################################################################
# 1.  Environment quirks when -u is active
###############################################################################
export AMENT_TRACE_SETUP_FILES=0          # stops tracing spam
export AMENT_PYTHON_EXECUTABLE=python3    # so local_setup.sh won't crash

###############################################################################
# 2.  Source ROS 2 – *with nounset temporarily off*
###############################################################################
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

###############################################################################
# 3.  Ensure workspace exists and perms are OK
###############################################################################
  mkdir -p "$WS/src"
chmod -R a+rwx "$WS" || true
cd "$WS"

###############################################################################
# 4.  Source the already-built workspace overlay
###############################################################################
if [ -f "install/setup.bash" ]; then
    log "Sourcing pre-built workspace overlay..."
    set +u
    source install/setup.bash
    set -u
    log "Workspace overlay sourced successfully."
else
    log "WARNING: No pre-built workspace found. You may need to rebuild the image."
fi

###############################################################################
# 5.  Hand control to CMD
###############################################################################
log "✓ Ready – executing CMD: $*"
exec "$@"
