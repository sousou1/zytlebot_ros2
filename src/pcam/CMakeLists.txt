cmake_minimum_required(VERSION 3.5)
project(pcam)

#set(CMAKE_CXX_FLAGS "-std=c++11 -lm -g -fsigned-char -fpermissive -Wint-to-pointer-cast -O3")

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    #add_compile_options(-Wall -Wextra -Wpedantic)
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(class_loader REQUIRED)

include_directories(include)
add_library(pcam_component SHARED src/pcam_component.cpp include/json_lib/json11.cpp)
target_compile_definitions(pcam_component PRIVATE "PCAM_BUILDING_DLL")
ament_target_dependencies(pcam_component
        sensor_msgs
        cv_bridge
        rclcpp
        class_loader
        std_msgs
        image_transport
        )

rclcpp_register_node_plugins(pcam_component "pcam::pcam")

include_directories(
        ${sensor_msgs_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${image_transport_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
)


# add_library(autorace src/autorace.cpp include/signal_lib/xcalc_hog.c include/signal_lib/xcalc_hog_linux.c  include/signal_lib/xcalc_hog_sinit.c include/signal_lib/linuxmisc.cpp include/signal_lib/linuxmmap.cpp include/json_lib/json11.cpp)
#add_definitions(-std=c++11)

# Build a standalone version of the node

add_executable(pcam src/pcam.cpp)
target_link_libraries(pcam pcam_component)

ament_target_dependencies(pcam
        sensor_msgs
        image_transport
        rclcpp
        std_msgs
        class_loader
        )
ament_export_include_directories(include)

ament_export_libraries(pcam_component)

install(DIRECTORY
        include/pcam
        DESTINATION include
        )
install(TARGETS
        pcam_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(TARGETS
        pcam
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()