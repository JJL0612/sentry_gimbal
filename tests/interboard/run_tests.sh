#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
reference_commit=0e533aff94d360dd755afa690fcfa471de33993d
reference_dir="${CHASSIS_SOURCE:-/tmp/sentry-onlychassis-reference-0e533af}"
if [[ ! -f "${reference_dir}/Skywalker/Algorithm/Src/alg_board_link.c" ]]; then
    if [[ -n "${CHASSIS_SOURCE:-}" ]]; then
        echo 'CHASSIS_SOURCE must point to a sentry_onlychassis checkout.' >&2
        exit 1
    fi
    reference_dir="$(mktemp -d /tmp/dm02-chassis-v4-reference.XXXXXX)"
    git -C "${reference_dir}" init -q
    git -C "${reference_dir}" fetch -q --depth 1 https://github.com/Bbeanana/sentry_onlychassis.git "${reference_commit}"
    git -C "${reference_dir}" checkout -q --detach FETCH_HEAD
fi
if [[ "$(git -C "${reference_dir}" rev-parse HEAD)" != "${reference_commit}" ]]; then
    echo "Reference must be sentry_onlychassis commit ${reference_commit}." >&2
    exit 1
fi
test_build_dir="$(mktemp -d /tmp/dm02-interboard-tests.XXXXXX)"
trap 'rm -rf -- "${test_build_dir}"' EXIT
flags=(-Wall -Wextra -Werror -fsanitize=address,undefined -g)
includes=(-I"${reference_dir}/Skywalker/Algorithm/Inc" -I"${reference_dir}/Skywalker/Application/Inc" -I"${project_dir}/tests/interboard/stubs")
gcc -std=c11 "${flags[@]}" "${includes[@]}" -c "${reference_dir}/Skywalker/Algorithm/Src/alg_board_link.c" -o "${test_build_dir}/codec.o"
gcc -std=c11 "${flags[@]}" "${includes[@]}" -c "${reference_dir}/Skywalker/Application/Src/app_gimbal_link.c" -o "${test_build_dir}/consumer.o"
g++ -std=c++17 "${flags[@]}" "${includes[@]}" -I"${project_dir}/User_File" \
    "${project_dir}/tests/interboard/link_tests.cpp" \
    "${project_dir}/User_File/2_Device/Communication/dvc_chassis_link.cpp" \
    "${test_build_dir}/codec.o" "${test_build_dir}/consumer.o" -o "${test_build_dir}/link_tests"
ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}" "${test_build_dir}/link_tests"
