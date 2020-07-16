cmake_minimum_required(VERSION 2.8.3)
project(tester)

add_definitions(-Wall -Wextra -Wmaybe-uninitialized -Wno-unused-result -Wno-unused-parameter -Werror -Wno-sign-compare)
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
#add_compile_options(-std=c++17)

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

## Generate messages in the 'msg' folder
#add_message_files(
#    FILES
#    rect.msg
#    optional_bbox_msg.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
#    FILES
#    optional_bbox.srv
#    register_policy_node.srv
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

#generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME}_lib
#  DEPENDS system_lib
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

include_directories(
    src
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_LIBS}
)

add_library(${PROJECT_NAME}_lib
    src/tester.cpp
)

#set_target_properties(${PROJECT_NAME}_lib PROPERTIES
#    CXX_STANDARD 17
#)

target_link_libraries(${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

# policies
#add_executable(seg_policy_node
#    src/segpolicynode.cpp
#)
#target_link_libraries(seg_policy_node
#    ${PROJECT_NAME}_lib
#)

# tester.cpp
add_executable(tester
    src/tester.cpp
)
target_link_libraries(tester
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
