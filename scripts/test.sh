#!/usr/bin/env bash
set -euo pipefail

export ROS_VERSION=2
export ROS_DISTRO="${ROS_DISTRO:=humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS setup not found at $ROS_SETUP"
  exit 1
fi

set +u
source "$ROS_SETUP"
set -u

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd -- "${REPO_ROOT}/../.." && pwd)"

echo "==> Repo root:      $REPO_ROOT"
echo "==> Workspace root: $WS_ROOT"

ONLY_PKGS=()
IGNORE_PKGS=()

usage() {
  cat <<EOF
Usage: $(basename "$0") [--only pkg1 pkg2 ...] [--ignore pkg1 pkg2 ...]

Options:
  --only     Test only the specified packages
  --ignore   Test all discovered packages except the specified ones

Examples:
  $0
  $0 --only nav_utils path_tracking
  $0 --ignore sensor_simulator
EOF
  exit 1
}

# ---- parse args ----
while [[ $# -gt 0 ]]; do
  case "$1" in
    --only)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        ONLY_PKGS+=("$1")
        shift
      done
      ;;
    --ignore)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        IGNORE_PKGS+=("$1")
        shift
      done
      ;;
    -h|--help)
      usage
      ;;
    *)
      echo "Unknown argument: $1"
      usage
      ;;
  esac
done

if [[ "${#ONLY_PKGS[@]}" -gt 0 && "${#IGNORE_PKGS[@]}" -gt 0 ]]; then
  echo "ERROR: --only and --ignore are mutually exclusive"
  exit 1
fi

cd "$WS_ROOT"

if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$WS_ROOT/install/setup.bash"
  set -u
else
  echo "ERROR: $WS_ROOT/install/setup.bash not found. Run scripts/build.sh first."
  exit 1
fi

echo "==> Discovering packages under repo root"
mapfile -t ALL_PKGS < <(colcon list --base-paths "$REPO_ROOT" --names-only)

if [[ "${#ALL_PKGS[@]}" -eq 0 ]]; then
  echo "ERROR: No colcon packages found under $REPO_ROOT"
  exit 1
fi

# ---- apply filters ----
if [[ "${#ONLY_PKGS[@]}" -gt 0 ]]; then
  PKGS=("${ONLY_PKGS[@]}")
elif [[ "${#IGNORE_PKGS[@]}" -gt 0 ]]; then
  PKGS=()
  for pkg in "${ALL_PKGS[@]}"; do
    skip=false
    for ignore in "${IGNORE_PKGS[@]}"; do
      if [[ "$pkg" == "$ignore" ]]; then
        skip=true
        break
      fi
    done
    $skip || PKGS+=("$pkg")
  done
else
  PKGS=("${ALL_PKGS[@]}")
fi

if [[ "${#PKGS[@]}" -eq 0 ]]; then
  echo "ERROR: No packages left to test after filtering"
  exit 1
fi

echo "==> colcon test (${#PKGS[@]} packages)"
colcon test --packages-select "${PKGS[@]}" --event-handlers console_direct+

echo "==> colcon test-result (verbose)"
colcon test-result --verbose
