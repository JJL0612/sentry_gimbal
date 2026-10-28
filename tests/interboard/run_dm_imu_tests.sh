#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
chassis_dir="${CHASSIS_SOURCE:-${project_dir}/../sentry_onlychassis}"
build_dir="$(mktemp -d /tmp/dm-imu-tests.XXXXXX)"
trap 'rm -rf -- "${build_dir}"' EXIT
flags=(-Wall -Wextra -Werror -fsanitize=address,undefined -g)
includes=(-I"${chassis_dir}/Skywalker/Algorithm/Inc" -I"${chassis_dir}/Skywalker/Application/Inc" -I"${project_dir}/tests/interboard/stubs")
gcc -std=c11 "${flags[@]}" "${includes[@]}" -c "${chassis_dir}/Skywalker/Algorithm/Src/alg_board_link.c" -o "${build_dir}/codec.o"
gcc -std=c11 "${flags[@]}" "${includes[@]}" -c "${chassis_dir}/Skywalker/Application/Src/app_gimbal_link.c" -o "${build_dir}/consumer.o"
g++ -std=c++17 "${flags[@]}" "${includes[@]}" -I"${project_dir}/User_File" \
 "${project_dir}/tests/interboard/dm_imu_tests.cpp" \
 "${project_dir}/User_File/2_Device/Communication/dvc_chassis_link.cpp" \
 "${project_dir}/User_File/2_Device/IMU/dvc_dm_imu.cpp" \
 "${build_dir}/codec.o" "${build_dir}/consumer.o" -o "${build_dir}/test"
ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}" "${build_dir}/test"
