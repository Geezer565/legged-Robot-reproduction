#!/usr/bin/env bash
set -Eeuo pipefail

fail() {
  printf '[adaptive-modules] ERROR: %s\n' "$*" >&2
  exit 1
}

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
repo_root="$(cd -- "${script_dir}/../../../.." && pwd -P)"
interface_root="${repo_root}/legged_interface"

[[ -f "${interface_root}/include/legged_interface/adaptive/AdaptiveTypes.h" ]] ||
  fail "cannot identify leggedcontrol_go2 root from ${script_dir}"

cxx="${CXX:-g++}"
command -v "${cxx}" >/dev/null 2>&1 || fail "C++ compiler not found: ${cxx}"

find_ocs2_include() {
  if [[ -n "${OCS2_CORE_INCLUDE:-}" ]]; then
    [[ -f "${OCS2_CORE_INCLUDE}/ocs2_core/Types.h" ]] ||
      fail "OCS2_CORE_INCLUDE does not contain ocs2_core/Types.h"
    printf '%s\n' "${OCS2_CORE_INCLUDE}"
    return
  fi

  local sibling="${repo_root}/../ocs2/ocs2_core/include"
  if [[ -f "${sibling}/ocs2_core/Types.h" ]]; then
    cd -- "${sibling}" && pwd -P
    return
  fi

  local types_header
  types_header="$(find -L "${repo_root}/.." -type f \
    -path '*/ocs2_core/include/ocs2_core/Types.h' -print -quit 2>/dev/null || true)"
  [[ -n "${types_header}" ]] ||
    fail "ocs2_core/include was not found; set OCS2_CORE_INCLUDE"
  dirname -- "$(dirname -- "${types_header}")"
}

find_eigen_include() {
  if [[ -n "${EIGEN3_INCLUDE_DIR:-}" ]]; then
    [[ -f "${EIGEN3_INCLUDE_DIR}/Eigen/Core" ]] ||
      fail "EIGEN3_INCLUDE_DIR does not contain Eigen/Core"
    printf '%s\n' "${EIGEN3_INCLUDE_DIR}"
    return
  fi

  if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists eigen3; then
    local flag candidate
    for flag in $(pkg-config --cflags-only-I eigen3); do
      candidate="${flag#-I}"
      if [[ -f "${candidate}/Eigen/Core" ]]; then
        printf '%s\n' "${candidate}"
        return
      fi
    done
  fi

  local candidate
  for candidate in /usr/include/eigen3 /usr/local/include/eigen3; do
    if [[ -f "${candidate}/Eigen/Core" ]]; then
      printf '%s\n' "${candidate}"
      return
    fi
  done
  fail "Eigen3 was not found; set EIGEN3_INCLUDE_DIR"
}

ocs2_include="$(find_ocs2_include)"
eigen_include="$(find_eigen_include)"

module_names=(
  PeriodicResidualModule
  PayloadIdModule
  ResidualDynamicsModule
  AdaptiveFeatureModule
  ResidualConfidenceModule
  AdaptiveModuleShadowPipeline
)

sources=()
tests=()
for module in "${module_names[@]}"; do
  source_file="${interface_root}/src/adaptive/modules/${module}.cpp"
  test_file="${interface_root}/test/adaptive/modules/Test${module}.cpp"
  header_file="${interface_root}/include/legged_interface/adaptive/modules/${module}.h"
  [[ -f "${header_file}" ]] || fail "missing module header: ${header_file}"
  [[ -f "${source_file}" ]] || fail "missing module source: ${source_file}"
  [[ -f "${test_file}" ]] || fail "missing module test: ${test_file}"
  sources+=("${source_file}")
  tests+=("${test_file}")
done

gtest_compile_flags=()
gtest_link_flags=()
gtest_sources=()

if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists gtest_main; then
  read -r -a gtest_compile_flags <<< "$(pkg-config --cflags gtest_main)"
  read -r -a gtest_link_flags <<< "$(pkg-config --libs gtest_main)"
elif [[ -n "${GTEST_ROOT:-}" && -f "${GTEST_ROOT}/include/gtest/gtest.h" ]]; then
  gtest_compile_flags+=("-isystem" "${GTEST_ROOT}/include")
  gtest_library="$(find "${GTEST_ROOT}" -type f -name 'libgtest.a' -print -quit 2>/dev/null || true)"
  gtest_main_library="$(find "${GTEST_ROOT}" -type f -name 'libgtest_main.a' -print -quit 2>/dev/null || true)"
  [[ -n "${gtest_library}" && -n "${gtest_main_library}" ]] ||
    fail "GTEST_ROOT has headers but not libgtest.a and libgtest_main.a"
  gtest_link_flags+=("${gtest_main_library}" "${gtest_library}" "-pthread")
elif [[ -f /usr/include/gtest/gtest.h ]]; then
  gtest_library="$(find /usr/lib /usr/local/lib -type f -name 'libgtest.a' -print -quit 2>/dev/null || true)"
  gtest_main_library="$(find /usr/lib /usr/local/lib -type f -name 'libgtest_main.a' -print -quit 2>/dev/null || true)"
  if [[ -n "${gtest_library}" && -n "${gtest_main_library}" ]]; then
    gtest_link_flags+=("${gtest_main_library}" "${gtest_library}" "-pthread")
  elif [[ -f /usr/src/googletest/googletest/src/gtest-all.cc ]]; then
    gtest_compile_flags+=("-isystem" "/usr/src/googletest/googletest/include" \
                         "-I/usr/src/googletest/googletest")
    gtest_sources+=("/usr/src/googletest/googletest/src/gtest-all.cc" \
                   "/usr/src/googletest/googletest/src/gtest_main.cc")
    gtest_link_flags+=("-pthread")
  elif [[ -f /usr/src/gtest/src/gtest-all.cc ]]; then
    gtest_compile_flags+=("-isystem" "/usr/src/gtest/include" "-I/usr/src/gtest")
    gtest_sources+=("/usr/src/gtest/src/gtest-all.cc" \
                   "/usr/src/gtest/src/gtest_main.cc")
    gtest_link_flags+=("-pthread")
  else
    fail "gtest headers were found, but neither libraries nor sources were found"
  fi
else
  fail "gtest was not found; install libgtest-dev or set GTEST_ROOT"
fi

build_dir="$(mktemp -d "${TMPDIR:-/tmp}/go2-adaptive-modules.XXXXXX")"
binary="${build_dir}/adaptive_module_tests"
cleanup() {
  if [[ "${KEEP_BUILD:-0}" == "1" ]]; then
    printf '[adaptive-modules] kept build directory: %s\n' "${build_dir}"
  else
    rm -rf -- "${build_dir}"
  fi
}
trap cleanup EXIT

printf '[adaptive-modules] repository : %s\n' "${repo_root}"
printf '[adaptive-modules] compiler   : %s\n' "$(command -v "${cxx}")"
printf '[adaptive-modules] OCS2       : %s\n' "${ocs2_include}"
printf '[adaptive-modules] Eigen      : %s\n' "${eigen_include}"
printf '[adaptive-modules] compiling M01-M05 and the unified pipeline...\n'

"${cxx}" \
  -std=c++14 \
  -O0 \
  -Wall \
  -Wextra \
  -Wpedantic \
  -Wshadow \
  -Werror \
  -fno-common \
  -pthread \
  -I"${interface_root}/include" \
  -I"${ocs2_include}" \
  -isystem "${eigen_include}" \
  "${gtest_compile_flags[@]}" \
  "${sources[@]}" \
  "${tests[@]}" \
  "${gtest_sources[@]}" \
  "${gtest_link_flags[@]}" \
  -o "${binary}"

printf '[adaptive-modules] running all module and pipeline tests...\n'
"${binary}" --gtest_color=yes "$@"
printf '[adaptive-modules] PASS: M01-M05 and the unified pipeline passed.\n'
