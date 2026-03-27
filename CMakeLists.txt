cmake_minimum_required(VERSION 3.22)


# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME rapid_drive_foc)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)


# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

    ${CMAKE_SOURCE_DIR}/Foc/FOC_Core/FOC.c
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Core/FOC_ADC.c
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Core/FOC_Diagnostics.c
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Core/FOC_Statemachine.c

    ${CMAKE_SOURCE_DIR}/Foc/FOC_Drivers/FOC_CAN.c
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Drivers/FOC_Flash.c
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Drivers/FOC_Utils.c

    ${CMAKE_SOURCE_DIR}/Foc/FOC_Loops/FOC_Alignment.c
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Loops/FOC_AlignmentTest.c
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Loops/FOC_AntiCogging.c
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Loops/FOC_ControlLoops.c
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Loops/FOC_CurrentSensorCalibration.c
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Loops/FOC_MotorIdentification.c
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Loops/FOC_OpenLoop.c
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Loops/FOC_PIDAutotune.c

    ${CMAKE_SOURCE_DIR}/Foc/Hardware_Drivers/AS5047P_Driver.c
    ${CMAKE_SOURCE_DIR}/Foc/Hardware_Drivers/DRV8323_Driver.c
    ${CMAKE_SOURCE_DIR}/Foc/Hardware_Drivers/WS2812b_Driver.c

    ${CMAKE_SOURCE_DIR}/Foc/Sequencers/SoundEngine.c
    ${CMAKE_SOURCE_DIR}/Foc/Sequencers/LightEngine.c

    ${CMAKE_SOURCE_DIR}/Foc/Utils/Logging.c
    ${CMAKE_SOURCE_DIR}/Foc/Utils/PID.c
    ${CMAKE_SOURCE_DIR}/Foc/Utils/Utils.c

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    # ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Core
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Drivers
    ${CMAKE_SOURCE_DIR}/Foc/FOC_Loops
    ${CMAKE_SOURCE_DIR}/Foc/Hardware_Drivers
    ${CMAKE_SOURCE_DIR}/Foc/Sequencers
    ${CMAKE_SOURCE_DIR}/Foc/Utils
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating binary file from executable"
)