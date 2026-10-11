#!/usr/bin/env bash
set -euo pipefail

readonly MUJOCO_VERSION="3.3.6"
readonly MUJOCO_SOURCE_COMMIT="eacad44a1a67afe520b263c9b15dab82f62a10aa"
readonly MUJOCO_SOURCE_SHA256="79ae9cceba4aaece8076220eee0dd7bc38ff809185f2f14e32283d66b1e62e8b"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "${script_dir}/.." && pwd)"
link_path="${repo_dir}/simulate/mujoco"
install_dir="${MUJOCO_INSTALL_DIR:-${HOME}/.mujoco/mujoco-${MUJOCO_VERSION}}"

usage() {
  cat <<EOF
Usage: $0 [--install-dir PATH]

Download the pinned MuJoCo ${MUJOCO_VERSION} binary and matching viewer sources,
then link simulate/mujoco to the resulting installation.

The default installation is:
  ${HOME}/.mujoco/mujoco-${MUJOCO_VERSION}

MUJOCO_INSTALL_DIR may also override the installation path.
EOF
}

while (($#)); do
  case "$1" in
    --install-dir)
      if (($# < 2)); then
        echo "error: --install-dir requires a path" >&2
        exit 2
      fi
      install_dir="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "error: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "error: the automated MuJoCo setup currently supports Linux only" >&2
  exit 1
fi

if [[ "${install_dir}" != /* ]]; then
  install_dir="$(pwd)/${install_dir}"
fi

for command in curl sha256sum tar; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "error: required command not found: ${command}" >&2
    exit 1
  fi
done

case "$(uname -m)" in
  x86_64|amd64)
    platform="linux-x86_64"
    binary_sha256="049204172901afad251070385a6badf46d795ebe47403d093f8469557eeeab5a"
    ;;
  aarch64|arm64)
    platform="linux-aarch64"
    binary_sha256="dd6389a008a1a4e4678b4a6118c0a099621f3f26721b6f05bb689a92250850b5"
    ;;
  *)
    echo "error: unsupported architecture: $(uname -m)" >&2
    exit 1
    ;;
esac

is_ready() {
  local root="$1"
  [[ -f "${root}/include/mujoco/mujoco.h" ]] &&
    [[ -f "${root}/simulate/simulate.cc" ]] &&
    { [[ -f "${root}/lib/libmujoco.so" ]] ||
      [[ -f "${root}/build/lib/libmujoco.so" ]]; }
}

if ! is_ready "${install_dir}"; then
  if [[ -e "${install_dir}" ]]; then
    echo "error: ${install_dir} exists but is not a complete MuJoCo installation" >&2
    echo "Move it aside or choose another path with --install-dir." >&2
    exit 1
  fi

  tmp_dir="$(mktemp -d)"
  trap 'rm -rf -- "${tmp_dir}"' EXIT

  binary_name="mujoco-${MUJOCO_VERSION}-${platform}.tar.gz"
  binary_url="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${binary_name}"
  binary_archive="${tmp_dir}/${binary_name}"
  source_archive="${tmp_dir}/mujoco-${MUJOCO_SOURCE_COMMIT}.tar.gz"
  source_url="https://github.com/google-deepmind/mujoco/archive/${MUJOCO_SOURCE_COMMIT}.tar.gz"

  echo "Downloading MuJoCo ${MUJOCO_VERSION} for ${platform}..."
  curl --location --fail --retry 3 --silent --show-error \
    --output "${binary_archive}" "${binary_url}"
  printf '%s  %s\n' "${binary_sha256}" "${binary_archive}" | sha256sum --check --status

  echo "Downloading matching MuJoCo viewer sources..."
  curl --location --fail --retry 3 --silent --show-error \
    --output "${source_archive}" "${source_url}"
  printf '%s  %s\n' "${MUJOCO_SOURCE_SHA256}" "${source_archive}" | sha256sum --check --status

  mkdir -p "${tmp_dir}/binary" "${tmp_dir}/source"
  tar -xzf "${binary_archive}" -C "${tmp_dir}/binary"
  tar -xzf "${source_archive}" -C "${tmp_dir}/source"

  payload="${tmp_dir}/binary/mujoco-${MUJOCO_VERSION}"
  source_root="${tmp_dir}/source/mujoco-${MUJOCO_SOURCE_COMMIT}"
  cp -a "${source_root}/simulate" "${payload}/simulate"

  if ! is_ready "${payload}"; then
    echo "error: downloaded MuJoCo payload is incomplete" >&2
    exit 1
  fi

  mkdir -p "$(dirname -- "${install_dir}")"
  mv "${payload}" "${install_dir}"
  echo "Installed MuJoCo at ${install_dir}"
else
  echo "MuJoCo ${MUJOCO_VERSION} is already ready at ${install_dir}"
fi

if [[ -e "${link_path}" && ! -L "${link_path}" ]]; then
  echo "error: ${link_path} exists and is not a symlink" >&2
  echo "Move it aside before running this setup script." >&2
  exit 1
fi

ln -sfn "${install_dir}" "${link_path}"
echo "Linked ${link_path} -> ${install_dir}"
echo
echo "Build with:"
echo "  cmake -S ${repo_dir}/simulate -B ${repo_dir}/simulate/build -DCMAKE_BUILD_TYPE=Release"
echo "  cmake --build ${repo_dir}/simulate/build -j\$(nproc)"
