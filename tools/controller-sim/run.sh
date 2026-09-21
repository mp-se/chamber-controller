#!/bin/bash
# Host tests and closed loop simulation for lib/brewpi-pid, see README.md.
#
#   run.sh test                  unit tests + closed loop regression check (exit code 0 = all fine)
#   run.sh compare [git-rev]     compare the working tree with the library at a git revision
#                                (default HEAD): summary metrics under perturbation + replay test
#   run.sh update-expected       rewrite expected/metrics.txt from the working tree
set -u

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
LIB="${SIM_LIB:-$REPO/lib/brewpi-pid}"  # SIM_LIB: test another copy of the library
BUILD="${SIM_BUILD:-$HERE/build}"
CXX="${CXX:-c++}"
CXXFLAGS="-std=c++17 -O1 -fno-rtti -DESP32 -Wall -Wextra -Wno-unused-parameter -Wno-unused-const-variable -I$HERE/stub"
SCENARIOS="fridge_cool beer_cool beer_heat beer_step beer_cool_noisy fridge_disconnect beer_heat_noisy
  nh_fridge_cool nh_beer_cool nh_beer_cool_noisy nh_beer_warm nh_beer_cold_room nh_beer_step nh_fridge_disconnect
  nc_fridge_heat nc_beer_heat nc_beer_heat_noisy nc_beer_cool nc_beer_step nc_fridge_disconnect
  fp_fridge_cool fp_fridge_heat fp_beer_cool bp_beer_cool bp_beer_heat bp_fridge_cool
  ble_beer_cool ble_beer_heat ble_beer_boot nh_ble_beer_cool nc_ble_beer_heat"
PERTURBATIONS="-0.20 -0.10 -0.05 0.00 0.05 0.10 0.20"

# The library files the host build needs: no OneWire sensor, no DallasTempNG, no digital pin.
# Revisions from before the floating point rewrite also have FilterFixed.cpp.
sim_sources() {
  for f in TempControl TempSensor TemperatureFormats FilterCascaded FilterFixed Ticks MinTimes ControlConstants ControlSettings; do
    [ -f "$1/$f.cpp" ] && echo "$1/$f.cpp"
  done
}

# sim.cpp adapts to the temperature type of the library
double_flag() {
  grep -q "typedef double temperature" "$1/TemperatureFormats.hpp" && echo "-DTEMP_IS_DOUBLE"
}

build_sim() { # <library dir> <output>
  # shellcheck disable=SC2046
  $CXX $CXXFLAGS $(double_flag "$1") -I"$1" "$HERE/src/sim.cpp" $(sim_sources "$1") -o "$2"
}

# The controller keeps state between runs, so every scenario gets its own process
run_scenarios() { # <binary> <sim arguments...>
  local bin=$1
  shift
  local s
  for s in $SCENARIOS; do
    SIM_ONLY=$s "$bin" "$@" || return 1
  done
}

unit() { # <name> <library sources...>: builds src/<name>.cpp against the working tree and runs it
  local name=$1
  shift
  echo "--- $name"
  $CXX $CXXFLAGS -DTEMP_IS_DOUBLE -I"$LIB" "$HERE/src/$name.cpp" "$@" -o "$BUILD/$name" && "$BUILD/$name"
}

cmd_test() {
  mkdir -p "$BUILD"
  local failed=0

  echo "--- filter against the exact solution"
  $CXX $CXXFLAGS -DTEMP_IS_DOUBLE -I"$LIB" "$HERE/src/filter_test.cpp" "$LIB/TemperatureFormats.cpp" -o "$BUILD/filter_test" &&
    "$BUILD/filter_test" steps >"$BUILD/filter_steps.txt" &&
    python3 "$HERE/scripts/filter_check.py" "$BUILD/filter_steps.txt" || failed=1

  unit sensor_test "$LIB/TempSensor.cpp" "$LIB/TemperatureFormats.cpp" || failed=1
  unit format_test "$LIB/NumberFormats.cpp" "$LIB/TemperatureFormats.cpp" || failed=1
  unit ticks_test "$LIB/Ticks.cpp" || failed=1

  echo "--- closed loop simulation against expected/metrics.txt"
  build_sim "$LIB" "$BUILD/sim" &&
    run_scenarios "$BUILD/sim" - 0 >"$BUILD/metrics.txt" &&
    python3 "$HERE/scripts/check_expected.py" "$HERE/expected/metrics.txt" "$BUILD/metrics.txt" || failed=1
  python3 "$HERE/scripts/check_actuator_rules.py" "$BUILD/metrics.txt" || failed=1
  python3 "$HERE/scripts/check_control.py" "$BUILD/metrics.txt" || failed=1

  echo
  if [ $failed -eq 0 ]; then echo "ALL CHECKS PASSED"; else echo "SOMETHING FAILED, see above"; fi
  return $failed
}

cmd_update_expected() {
  mkdir -p "$BUILD" "$HERE/expected"
  build_sim "$LIB" "$BUILD/sim" &&
    run_scenarios "$BUILD/sim" - 0 >"$HERE/expected/metrics.txt" &&
    echo "wrote $HERE/expected/metrics.txt"
}

cmd_compare() {
  local rev=${1:-HEAD}
  local root="$BUILD/ref"
  rm -rf "$root" "$BUILD"/var_ref "$BUILD"/var_cur "$BUILD"/reads "$BUILD"/trace_live "$BUILD"/trace_ref "$BUILD"/trace_cur
  mkdir -p "$root" "$BUILD"/var_ref "$BUILD"/var_cur "$BUILD"/reads "$BUILD"/trace_live "$BUILD"/trace_ref "$BUILD"/trace_cur

  git -C "$REPO" archive "$rev" lib/brewpi-pid | tar -x -C "$root" || { echo "cannot read lib/brewpi-pid at $rev"; return 2; }
  local ref="$root/lib/brewpi-pid"
  build_sim "$ref" "$BUILD/sim_ref" || return 2
  build_sim "$LIB" "$BUILD/sim_cur" || return 2

  local p
  for p in $PERTURBATIONS; do
    run_scenarios "$BUILD/sim_ref" - "$p" >"$BUILD/var_ref/m_$p.txt" || return 2
    run_scenarios "$BUILD/sim_cur" - "$p" >"$BUILD/var_cur/m_$p.txt" || return 2
  done
  echo "=== summary metrics: $rev (reference) against the working tree, perturbed start temperatures"
  python3 "$HERE/scripts/compare_metrics.py" "$BUILD/var_ref" "$BUILD/var_cur"
  local status=$?
  python3 "$HERE/scripts/check_actuator_rules.py" "$BUILD"/var_ref/m_*.txt "$BUILD"/var_cur/m_*.txt || status=1
  python3 "$HERE/scripts/check_control.py" "$BUILD"/var_cur/m_*.txt || status=1

  # Record the sensor readings the reference saw in its closed loop run and feed exactly those into
  # both controllers with the loop open: any difference is numerical
  run_scenarios "$BUILD/sim_ref" "$BUILD/trace_live" 0 record "$BUILD/reads" >/dev/null || return 2
  run_scenarios "$BUILD/sim_ref" "$BUILD/trace_ref" 0 replay "$BUILD/reads" >/dev/null || return 2
  run_scenarios "$BUILD/sim_cur" "$BUILD/trace_cur" 0 replay "$BUILD/reads" >/dev/null || return 2
  echo
  echo "=== replay of identical sensor input: $rev (reference) against the working tree"
  python3 "$HERE/scripts/compare_replay.py" "$BUILD/trace_ref" "$BUILD/trace_cur" || status=1
  return $status
}

case "${1:-}" in
  test) cmd_test ;;
  compare) cmd_compare "${2:-HEAD}" ;;
  update-expected) cmd_update_expected ;;
  *)
    sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
