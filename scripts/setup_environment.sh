#!/usr/bin/env bash
# One-click environment setup for this workspace (Ubuntu + ROS 2).
# Usage:
#   ./scripts/setup_environment.sh
#   ROS_DISTRO=jazzy ./scripts/setup_environment.sh
#   SKIP_APT=1 ./scripts/setup_environment.sh   # only pip + rosdep
#   SKIP_PIP=1 ./scripts/setup_environment.sh   # apt + rosdep only
#   UPGRADE_PIP=1 ./scripts/setup_environment.sh
#
# Pip mirror (default: Tsinghua; override if needed):
#   PIP_INDEX_URL=https://mirrors.aliyun.com/pypi/simple ./scripts/setup_environment.sh
# rosdep index (default: Tsinghua; raw.githubusercontent.com often times out):
#   ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml ./scripts/setup_environment.sh
#   SKIP_ROSDEP=1 ./scripts/setup_environment.sh

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
APT_FILE="${ROOT}/requirements-apt.txt"
PIP_FILE="${ROOT}/requirements.txt"

# Prefer a mainland mirror; files.pythonhosted.org often times out.
PIP_INDEX_URL="${PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"
PIP_TRUSTED_HOST="${PIP_TRUSTED_HOST:-pypi.tuna.tsinghua.edu.cn}"
PIP_DEFAULT_TIMEOUT="${PIP_DEFAULT_TIMEOUT:-120}"

# Prefer a mainland rosdistro index (required by rosdep update).
export ROSDISTRO_INDEX_URL="${ROSDISTRO_INDEX_URL:-https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml}"
ROSDEP_SOURCES_LIST_URL="${ROSDEP_SOURCES_LIST_URL:-https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list}"

echo "==> ROS_DISTRO=${ROS_DISTRO}"
echo "==> Workspace: ${ROOT}"

if [[ "${SKIP_APT:-0}" != "1" ]]; then
  if ! command -v apt-get >/dev/null 2>&1; then
    echo "apt-get not found; set SKIP_APT=1 or run on Debian/Ubuntu." >&2
    exit 1
  fi

  echo "==> Updating apt indexes"
  sudo apt-get update -y

  pkgs=()
  while IFS= read -r line || [[ -n "${line}" ]]; do
    # trim
    line="${line%%#*}"
    line="$(echo "${line}" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')"
    [[ -z "${line}" ]] && continue

    if [[ "${line}" == ROS_PKG:* ]]; then
      name="${line#ROS_PKG:}"
      pkgs+=("ros-${ROS_DISTRO}-${name}")
    else
      pkgs+=("${line}")
    fi
  done < "${APT_FILE}"

  echo "==> Installing ${#pkgs[@]} apt packages"
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${pkgs[@]}"
fi

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # ROS setup scripts reference optional unset vars; incompatible with `set -u`.
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
else
  echo "WARN: /opt/ros/${ROS_DISTRO}/setup.bash missing; ROS packages may be incomplete." >&2
fi

if [[ "${SKIP_PIP:-0}" != "1" ]]; then
  echo "==> Installing pip requirements (index: ${PIP_INDEX_URL})"
  pip_common=(
    --user
    -i "${PIP_INDEX_URL}"
    --trusted-host "${PIP_TRUSTED_HOST}"
    --default-timeout="${PIP_DEFAULT_TIMEOUT}"
    --retries 5
  )
  # Do not upgrade pip by default: system pip from apt is enough and avoids
  # a large download that frequently times out on slow links.
  if [[ "${UPGRADE_PIP:-0}" == "1" ]]; then
    python3 -m pip install "${pip_common[@]}" -U pip
  fi
  python3 -m pip install "${pip_common[@]}" -r "${PIP_FILE}"
else
  echo "==> SKIP_PIP=1; skipping pip install"
fi

if [[ "${SKIP_ROSDEP:-0}" != "1" ]] && command -v rosdep >/dev/null 2>&1; then
  ROSDEP_LIST="/etc/ros/rosdep/sources.list.d/20-default.list"
  if [[ ! -f "${ROSDEP_LIST}" ]]; then
    echo "==> rosdep init (mirror: ${ROSDEP_SOURCES_LIST_URL})"
    sudo mkdir -p "$(dirname "${ROSDEP_LIST}")"
    if command -v curl >/dev/null 2>&1; then
      sudo curl -fsSL -o "${ROSDEP_LIST}" "${ROSDEP_SOURCES_LIST_URL}" || sudo rosdep init || true
    else
      sudo rosdep init || true
    fi
  fi
  echo "==> rosdep update (ROSDISTRO_INDEX_URL=${ROSDISTRO_INDEX_URL})"
  if ! rosdep update; then
    echo "WARN: rosdep update failed. Apt deps from requirements-apt.txt are usually enough;" >&2
    echo "      retry with another mirror or SKIP_ROSDEP=1." >&2
  else
    echo "==> rosdep install (from src/)"
    rosdep install --from-paths "${ROOT}/src" --ignore-src -r -y || true
  fi
elif [[ "${SKIP_ROSDEP:-0}" == "1" ]]; then
  echo "==> SKIP_ROSDEP=1; skipping rosdep"
else
  echo "WARN: rosdep not found; skipped." >&2
fi

echo ""
echo "Done. Next:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  cd ${ROOT} && colcon build --symlink-install"
echo "  source install/setup.bash"
