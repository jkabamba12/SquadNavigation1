cmake_minimum_required(VERSION 3.2)
project(navigator)

include(CMakeListsPrivate.txt)
include_directories(".piolibdeps/sharp-distance/src")
include_directories(".piolibdeps/Dynamic_Motor_Driver/src")
include_directories(".piolibdeps/Encoder")
include_directories(".piolibdeps/qtr-sensors-arduino/QTRSensors")

add_custom_target(
        PLATFORMIO_BUILD ALL
        COMMAND ${PLATFORMIO_CMD} -f -c clion run
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
        PLATFORMIO_UPLOAD ALL
        COMMAND ${PLATFORMIO_CMD} -f -c clion run --target upload
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
        PLATFORMIO_CLEAN ALL
        COMMAND ${PLATFORMIO_CMD} -f -c clion run --target clean
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
        PLATFORMIO_TEST ALL
        COMMAND ${PLATFORMIO_CMD} -f -c clion test
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
        PLATFORMIO_PROGRAM ALL
        COMMAND ${PLATFORMIO_CMD} -f -c clion run --target program
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
        PLATFORMIO_UPLOADFS ALL
        COMMAND ${PLATFORMIO_CMD} -f -c clion run --target uploadfs
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
        PLATFORMIO_UPDATE_ALL ALL
        COMMAND ${PLATFORMIO_CMD} -f -c clion update
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
        PLATFORMIO_REBUILD_PROJECT_INDEX ALL
        COMMAND ${PLATFORMIO_CMD} -f -c clion init --ide clion
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

set(NAVIGATOR_SRC src/Navigator.h
        src/SharpWallNavigator/SharpWallNavigator.cpp src/SharpWallNavigator/SharpWallNavigator.h
        src/EncoderDriver/EncoderDriver.h src/EncoderDriver/EncoderDriver.cpp
        src/PID/PID.h src/PID/PID.cpp)

add_executable(${PROJECT_NAME} ${ENC_SRC} ${MOTOR_SRC} ${NAVIGATOR_SRC})
