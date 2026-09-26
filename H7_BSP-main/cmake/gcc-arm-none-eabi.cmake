set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# GNU Tools for STM32 installation. It can still be overridden from the
# command line with -DARM_TOOLCHAIN_DIR=<path>.
set(ARM_TOOLCHAIN_DIR
    "C:/ST/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin"
    CACHE PATH "GNU Arm Embedded toolchain binary directory")
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

find_program(CMAKE_C_COMPILER
    NAMES ${TOOLCHAIN_PREFIX}gcc
    HINTS "${ARM_TOOLCHAIN_DIR}"
    REQUIRED NO_DEFAULT_PATH)
set(CMAKE_ASM_COMPILER              "${CMAKE_C_COMPILER}")
find_program(CMAKE_CXX_COMPILER
    NAMES ${TOOLCHAIN_PREFIX}g++
    HINTS "${ARM_TOOLCHAIN_DIR}"
    REQUIRED NO_DEFAULT_PATH)
find_program(CMAKE_LINKER
    NAMES ${TOOLCHAIN_PREFIX}g++
    HINTS "${ARM_TOOLCHAIN_DIR}"
    REQUIRED NO_DEFAULT_PATH)
find_program(CMAKE_OBJCOPY
    NAMES ${TOOLCHAIN_PREFIX}objcopy
    HINTS "${ARM_TOOLCHAIN_DIR}"
    REQUIRED NO_DEFAULT_PATH)
find_program(CMAKE_SIZE
    NAMES ${TOOLCHAIN_PREFIX}size
    HINTS "${ARM_TOOLCHAIN_DIR}"
    REQUIRED NO_DEFAULT_PATH)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fdiagnostics-color=always")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics -fdiagnostics-color=always")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32H723XG_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
