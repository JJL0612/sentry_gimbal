# Find the toolchain via ARM_TOOLCHAIN_ROOT, PATH, or the local installations.
set(ARM_TOOLCHAIN_ROOT "" CACHE PATH "Arm GNU toolchain installation directory")
find_program(DM02_ARM_GCC arm-none-eabi-gcc
    HINTS
        "${ARM_TOOLCHAIN_ROOT}/bin"
        "$ENV{ARM_TOOLCHAIN_ROOT}/bin"
        "${CMAKE_CURRENT_LIST_DIR}/../.tools/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin"
        "${CMAKE_CURRENT_LIST_DIR}/../../dm02_test - 副本/.tools/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin"
    REQUIRED)
get_filename_component(ARM_TOOLCHAIN_BIN_DIR "${DM02_ARM_GCC}" DIRECTORY)

set(CMAKE_C_COMPILER "${ARM_TOOLCHAIN_BIN_DIR}/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "${ARM_TOOLCHAIN_BIN_DIR}/arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER "${CMAKE_C_COMPILER}")
set(CMAKE_AR "${ARM_TOOLCHAIN_BIN_DIR}/arm-none-eabi-ar")
set(CMAKE_RANLIB "${ARM_TOOLCHAIN_BIN_DIR}/arm-none-eabi-ranlib")
set(CMAKE_OBJCOPY "${ARM_TOOLCHAIN_BIN_DIR}/arm-none-eabi-objcopy")
set(CMAKE_OBJDUMP "${ARM_TOOLCHAIN_BIN_DIR}/arm-none-eabi-objdump")
set(SIZE "${ARM_TOOLCHAIN_BIN_DIR}/arm-none-eabi-size")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
