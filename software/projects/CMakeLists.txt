cmake_minimum_required(VERSION 3.16.3)

set(PROJECT_NAME "firmware")

set(CMAKE_SYSTEM_NAME             Linux)
set(CMAKE_SYSTEM_PROCESSOR        arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_ASM_COMPILER            arm-none-eabi-gcc)
set(CMAKE_C_COMPILER              arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER            arm-none-eabi-g++)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

project(${PROJECT_NAME} LANGUAGES C CXX ASM)

# C preprocessor definitions
set(DEFS -DVA416xx)                                                             

# C compile options
set(C_FLAGS -mcpu=cortex-m4)                                                        # arm cortex m4 cpu
list(APPEND C_FLAGS -mthumb)                                                        # use thumb instruction set
list(APPEND C_FLAGS -g3)                                                            # gdb level
list(APPEND C_FLAGS "-${OPT}")                                                      # optimization level
list(APPEND C_FLAGS -ffunction-sections)                                            # place each function into its own section
list(APPEND C_FLAGS -fdata-sections)                                                # place data items into their own section
list(APPEND C_FLAGS -Wall)                                                          # warnings options
list(APPEND C_FLAGS -Wstack-usage=1800)                                             # generate warnings if stack usage exceeds 1200 bytes
list(APPEND C_FLAGS -fstack-usage)                                                  # output stack usage information on a per-function basis
list(APPEND C_FLAGS -mfpu=fpv4-sp-d16)                                              # cpu has double precision Floating-Point Extension (FPv4)
list(APPEND C_FLAGS -mfloat-abi=hard)                                               # use floating point hardware
list(APPEND C_FLAGS -std=gnu11)                                                     # Conform to the ISO 2011 C standard
list(APPEND C_FLAGS --specs=nano.specs)                                             # use newlib nano

# C++ compile options
set(CXX_FLAGS -mcpu=cortex-m4)                                                      # arm cortex m4 cpu
list(APPEND CXX_FLAGS -mthumb)                                                      # use thumb instruction set
list(APPEND CXX_FLAGS -g3)                                                          # debug level
list(APPEND CXX_FLAGS "-${OPT}")                                                    # optimization level
list(APPEND CXX_FLAGS -ffunction-sections)                                          # place each function into its own section
list(APPEND CXX_FLAGS -fno-use-cxa-atexit)                                          # do not use call destructors i.e use __cxa_atexit() code
list(APPEND CXX_FLAGS -Wall)                                                        # warnings options
list(APPEND CXX_FLAGS -Wstack-usage=256)                                            # generate warnings if stack usage exceeds 256 bytes
list(APPEND CXX_FLAGS -fstack-usage)                                                # output stack usage information on a per-function basis
list(APPEND CXX_FLAGS --specs=nano.specs)                                           # use newlib nano
list(APPEND CXX_FLAGS -mfpu=fpv4-sp-d16)                                            # cpu has double precision Floating-Point Extension (FPv4)
list(APPEND CXX_FLAGS -mfloat-abi=hard)                                             # use floating point hardware
list(APPEND CXX_FLAGS -fno-exceptions)                                              # disable exceptions
list(APPEND CXX_FLAGS -fno-rtti)                                                    # disable run-time type information
list(APPEND CXX_FLAGS -std=gnu++11)                                                 # Conform to the ISO 2011 C++ standard

#
set(ASM_DEFS -DDEBUG)

# ASM options
set(ASM_FLAGS -mcpu=cortex-m4)                                                      # arm cortex m4 cpu
list(APPEND ASM_FLAGS -mthumb)                                                      # use thumb instruction set
list(APPEND ASM_FLAGS -g3)                                                          # debug level
list(APPEND ASM_FLAGS -mfpu=fpv4-sp-d16)                                            # cpu has double precision Floating-Point Extension (FPv4)
list(APPEND ASM_FLAGS -mfloat-abi=hard)                                             # use floating point hardware

# Linker options
set(LINK_FLAGS -mcpu=cortex-m4)                                                     # arm cortex m4 cpu
list(APPEND LINK_FLAGS -mthumb)                                                     # use thumb instruction set
list(APPEND LINK_FLAGS -Wl,--gc-sections)                                           # remove unused sections
list(APPEND LINK_FLAGS -static)                                                     # static link
list(APPEND LINK_FLAGS --specs=nosys.specs)                                         # system calls implemented as stubs
list(APPEND LINK_FLAGS --specs=nano.specs)                                          # use newlib nano
list(APPEND LINK_FLAGS -mfpu=fpv4-sp-d16)                                           # cpu has double precision Floating-Point Extension (FPv4)
list(APPEND LINK_FLAGS -mfloat-abi=hard)                                            # use floating point hardware
list(APPEND LINK_FLAGS -Wl,--start-group -lc -lm -Wl,--end-group)	                # link libc, libm, libstdc++ and libsupc++
list(APPEND LINK_FLAGS -Wl,--print-memory-usage)                                    # list detailed memory usage per memory section

# Loader Linker options
set(LD_LINK_FLAGS -mcpu=cortex-m4)                                                  # arm cortex m4 cpu
list(APPEND LD_LINK_FLAGS -mthumb)                                                  # use thumb instruction set
list(APPEND LD_LINK_FLAGS -nostdlib)                                                # do not link standard libraries
list(APPEND LD_LINK_FLAGS -mfpu=fpv4-sp-d16)                                        # cpu has double precision Floating-Point Extension (FPv4)
list(APPEND LD_LINK_FLAGS -mfloat-abi=hard)                                         # use floating point hardware
list(APPEND LD_LINK_FLAGS -Wl,--print-memory-usage)                                 # list detailed memory usage per memory section

# Linker script
set(LINK_SCRIPT -T${CMAKE_CURRENT_SOURCE_DIR}/va416xx.ld)
set(LD_LiNK_SCRIPT -T${CMAKE_CURRENT_SOURCE_DIR}/va416xx_fram.ld)

# Common Source paths
add_subdirectory(common/drivers/src)
add_subdirectory(common/mcu/src)
add_subdirectory(common/utils/src)
add_subdirectory(common/BSP/evk/src)

# Common Include paths
include_directories(common/BSP/evk/hdr)
include_directories(common/drivers/hdr)
include_directories(common/mcu/hdr)
include_directories(common/utils/hdr)

# freeRTOS Source path
add_subdirectory(freeRTOS/Source)

# freeRTOS Include path
include_directories(freeRTOS/Source/config)
include_directories(freeRTOS/Source/Include)
include_directories(freeRTOS/Source/portable/GCC/ARM_CM4F)

# Apps Source paths
add_subdirectory(apps/bootloader/src)
add_subdirectory(apps/loader)
add_subdirectory(apps/eraser)
add_subdirectory(apps/blinky/src)
add_subdirectory(apps/freertos_blinky/src)
add_subdirectory(apps/demo/src)
add_subdirectory(apps/ft_vor/src)

# Modify your build_target function
function(build_target target src)
    add_executable(${target} "${${src}}" "${COMMON_SRC}")
    
    # For ft_vor, create a target-specific build info generator
    if("${target}" STREQUAL "ft_vor")
        add_custom_target(
            ${target}_build_info
            COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_build_info.sh ${CMAKE_CURRENT_SOURCE_DIR}/apps/${target}
            COMMENT "Generating build information for ${target}..."
            VERBATIM
        )
        add_dependencies(${target} ${target}_build_info)
    endif()
    
    add_dependencies(${target} loader eraser)
	target_include_directories(${target} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/apps/${target}/hdr>
  )
  
	target_compile_options(${target} PUBLIC 
    $<$<COMPILE_LANGUAGE:C>:${DEFS} ${C_FLAGS}>
    $<$<COMPILE_LANGUAGE:CXX>:${DEFS} ${CXX_FLAGS}>
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp ${ASM_DEFS} ${ASM_FLAGS}>
)
	target_link_options(${target} PUBLIC "-Wl,-Map=${target}.map" ${LINK_FLAGS} ${LINK_SCRIPT})
	set_target_properties(${target} PROPERTIES SUFFIX ".elf")

	# Post build commands
	add_custom_command(TARGET ${target} POST_BUILD COMMAND arm-none-eabi-size "${target}.elf")
	add_custom_command(TARGET ${target} POST_BUILD COMMAND arm-none-eabi-objcopy  -O ihex  "${target}.elf"  "${target}.hex" )
endfunction()

function(build_freertos_target target src)
	add_executable(${target} "${${src}}" "${COMMON_SRC}" "${FREERTOS_SRC}")
	add_dependencies(${target} loader eraser)
	target_include_directories(${target} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/apps/${target}/hdr>
  )
  
	target_compile_options(${target} PUBLIC 
    $<$<COMPILE_LANGUAGE:C>:${DEFS} ${C_FLAGS}>
    $<$<COMPILE_LANGUAGE:CXX>:${DEFS} ${CXX_FLAGS}>
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp ${ASM_DEFS} ${ASM_FLAGS}>
)
	target_link_options(${target} PUBLIC "-Wl,-Map=${target}.map" ${LINK_FLAGS} ${LINK_SCRIPT})
	set_target_properties(${target} PROPERTIES SUFFIX ".elf")

	# Post build commands
	add_custom_command(TARGET ${target} POST_BUILD COMMAND arm-none-eabi-size "${target}.elf")
	add_custom_command(TARGET ${target} POST_BUILD COMMAND arm-none-eabi-objcopy  -O ihex  "${target}.elf"  "${target}.hex" )
endfunction()

function(build_loader loader src)
	add_executable(${loader} "${${src}}")

	target_compile_options(${loader} PUBLIC 
    $<$<COMPILE_LANGUAGE:C>:${DEFS} ${C_FLAGS}>
)
	target_link_options(${loader} PUBLIC "-Wl,-Map=${loader}.map" ${LD_LINK_FLAGS} ${LD_LiNK_SCRIPT})
	set_target_properties(${loader} PROPERTIES SUFFIX ".elf")
endfunction()

build_target(bootloader BOOTLOADER_SRC)

build_loader(loader LOADER_SRC)
build_loader(eraser ERASER_SRC)

build_target(blinky BLINKY_SRC)
build_freertos_target(freertos_blinky FREERTOS_BLINKY_SRC)
build_target(demo DEMO_SRC)
build_target(ft_vor FT_VOR_SRC)


