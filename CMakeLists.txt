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

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

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
    ${CMAKE_SOURCE_DIR}/Drivers/FOC_drivers/Src/FOC_Flash.c
    ${CMAKE_SOURCE_DIR}/Core/Src/FOC.c
    ${CMAKE_SOURCE_DIR}/Core/Src/FOC_CAN.c
    
    ${CMAKE_SOURCE_DIR}/Core/Src/FOC_Loops/FOC_MotorIdentification.c
    ${CMAKE_SOURCE_DIR}/Core/Src/FOC_Loops/FOC_OpenLoop.c
    ${CMAKE_SOURCE_DIR}/Core/Src/FOC_Loops/FOC_Alignment.c
    ${CMAKE_SOURCE_DIR}/Core/Src/FOC_Loops/FOC_BootupSound.c
    ${CMAKE_SOURCE_DIR}/Core/Src/FOC_Loops/FOC_PIDAutotune.c
    ${CMAKE_SOURCE_DIR}/Core/Src/FOC_Loops/FOC_CurrentSensorCalibration.c
    ${CMAKE_SOURCE_DIR}/Core/Src/FOC_Loops/FOC_AlignmentTest.c
    ${CMAKE_SOURCE_DIR}/Core/Src/FOC_Loops/FOC_AntiCogging.c
    ${CMAKE_SOURCE_DIR}/Core/Src/FOC_Loops/FOC_ControlLoops.c


    ${CMAKE_SOURCE_DIR}/Drivers/FOC_drivers/Src/FOC_Driver.c
    ${CMAKE_SOURCE_DIR}/Drivers/FOC_drivers/Src/PID.c
    ${CMAKE_SOURCE_DIR}/Drivers/FOC_drivers/Src/DRV8323_Driver.c
    ${CMAKE_SOURCE_DIR}/Drivers/FOC_drivers/Src/AS5047P_Driver.c
    ${CMAKE_SOURCE_DIR}/Drivers/FOC_drivers/Src/FOC_Utils.c
    ${CMAKE_SOURCE_DIR}/Drivers/FOC_drivers/Src/WS2812b_Driver.c
    ${CMAKE_SOURCE_DIR}/Drivers/FOC_drivers/Src/Logging.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/FOC_drivers/Inc
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
