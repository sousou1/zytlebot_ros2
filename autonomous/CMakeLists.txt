cmake_minimum_required(VERSION 2.8.3)
project(autonomous)

set(CMAKE_CXX_FLAGS "-std=c++11 -lm -g -fsigned-char -fpermissive -Wint-to-pointer-cast -O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(class_loader REQUIRED)

target_compile_definitions(autonomous_component PROVATE "")
ament_target_dependencies(autonomous_component
        sensor_msgs
        cv_bridge
        rclcpp
        class_loader
        std_msgs
        image_transport
        geometry_msgs
        )

rclcpp_register_node_plugins(autonomous_component "autonomous::Autonomous")

include_directories(
        ${sensor_msgs_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${image_transport_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
)


# add_library(autorace src/autorace.cpp include/signal_lib/xcalc_hog.c include/signal_lib/xcalc_hog_linux.c  include/signal_lib/xcalc_hog_sinit.c include/signal_lib/linuxmisc.cpp include/signal_lib/linuxmmap.cpp include/json_lib/json11.cpp)
add_library(autonomous src/autonomous.cpp include/json_lib/json11.cpp)
#add_definitions(-std=c++11)

# Build a standalone version of the node

add_executable(autonomous src/autonomous.cpp)
target_link_libraries(autonomous autonomous_component)

ament_target_dependencies(autonomous
        sensor_msgs
        cv_bridge
        rclcpp
        std_msgs
        class_loader
        image_transport
        geometry_msgs
        )

install(DIRECTORY
        include/autonomous
        DESTINATION include
        )
install(TARGETS
        autonomous_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(TARGETS
        autonomous
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()