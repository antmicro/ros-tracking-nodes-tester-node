cmake_minimum_required(VERSION 2.8.3)
project(tracking_tester)

add_definitions(-Wall -Wextra -Wmaybe-uninitialized -Wno-unused-result -Wno-unused-parameter -Werror -Wno-sign-compare)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    sensor_msgs
    std_msgs
    message_generation
)

execute_process(
    COMMAND git log -1 --format=%h
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_definitions(-DCOMMIT="${GIT_COMMIT_HASH}")

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME}_lib
 	DEPENDS system_lib
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

include_directories(
    src
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_LIBS}
)

add_library(${PROJECT_NAME}_lib
    src/tracking_tester.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

# tracking_tester.cpp
add_executable(tracking_tester
    src/tracking_tester.cpp
)
target_link_libraries(tracking_tester
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
