cmake_minimum_required(VERSION 2.8.3)
project(tracking_tester)

add_definitions(-Wall -Wextra -Wmaybe-uninitialized -Wno-unused-result -Wno-unused-parameter -Werror)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    sensor_msgs
    std_msgs
    message_generation
    stopwatch
)

execute_process(
    COMMAND git log -1 --format=%h
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_definitions(-DCOMMIT="${GIT_COMMIT_HASH}")

add_message_files(DIRECTORY msg FILES frame_msg.msg)
add_message_files(DIRECTORY msg FILES optional_bbox_msg.msg)
add_message_files(DIRECTORY msg FILES rect.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME}_lib
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(
    src
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_LIBS}
)

add_executable(tracking_tester
    src/tracking_tester.cpp
    src/FileSystemUtils.cpp
)

add_dependencies(tracking_tester ${PROJECT_NAME}_gencpp)

target_link_libraries(tracking_tester
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

add_library(tracking_tester_lib
    src/tracking_tester.cpp
    src/FileSystemUtils.cpp
)

add_dependencies(tracking_tester_lib ${PROJECT_NAME}_gencpp)

target_link_libraries(tracking_tester_lib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
