cmake_minimum_required(VERSION 3.5)
project(yolo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # add_compile_options(-lm -g -fsigned-char -fpermissive -Wint-to-pointer-cast -O3)
endif()


if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_INSTALL_RPATH "include/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(class_loader REQUIRED)

include_directories(include)


add_library(yolo_component SHARED src/yolo_component.cpp)
# 最適化・警告等のオプション
target_compile_options(yolo_component PUBLIC -c -O2 -Wall -Wpointer-arith -ffast-math)
# C++11/14/17
target_compile_definitions(yolo_component PRIVATE "YOLO_BUILDING_DLL")
# target_compile_features(yolo_component PUBLIC cxx_std_11)



ament_target_dependencies(yolo_component
        sensor_msgs
        cv_bridge
        rclcpp
        rclcpp_components
        class_loader
        std_msgs
        image_transport
        )

rclcpp_components_register_nodes(yolo_component "yolo::yolo")

include_directories(
        ${sensor_msgs_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${rclcpp_components_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${image_transport_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
        include/dnndk
)


# add_library(autorace src/autorace.cpp include/signal_lib/xcalc_hog.c include/signal_lib/xcalc_hog_linux.c  include/signal_lib/xcalc_hog_sinit.c include/signal_lib/linuxmisc.cpp include/signal_lib/linuxmmap.cpp include/json_lib/json11.cpp)
#add_definitions(-std=c++11)

# Build a standalone version of the node

add_executable(yolo include/model/dpu_yolo.elf src/yolo.cpp)
target_link_libraries(yolo yolo_component hineon n2cube dputils)
link_directories(include/bin include/lib)
target_compile_options(yolo PUBLIC -O2 -Wall -Wpointer-arith -ffast-math)
# target_compile_features(yolo PUBLIC cxx_std_11)



ament_target_dependencies(yolo
        sensor_msgs
        image_transport
        rclcpp
        std_msgs
        class_loader
        )
ament_export_include_directories(include)

ament_export_libraries(yolo_component)

install(DIRECTORY
        include/yolo
        include/dnndk
        DESTINATION include
        )
install(TARGETS
        yolo_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(TARGETS
        yolo
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()