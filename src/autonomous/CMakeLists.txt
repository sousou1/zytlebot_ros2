cmake_minimum_required(VERSION 3.5)
project(autonomous)

#set(CMAKE_CXX_FLAGS "-std=c++11 -lm -g -fsigned-char -fpermissive -Wint-to-pointer-cast -O3")

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_FLAGS "-lm -g -fsigned-char -fpermissive -Wint-to-pointer-cast -O3")


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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(class_loader REQUIRED)

include_directories(include)

# コンポーネントノードの共有ライブラリをコンパイルする
add_library(autonomous_component SHARED src/autonomous_component.cpp include/json_lib/json11.cpp)
# コンポーネントノードのヘッダーファイルのマクロを設定する
target_compile_definitions(autonomous_component PRIVATE "AUTONOMOUS_BUILDING_DLL")
ament_target_dependencies(autonomous_component
        sensor_msgs
        cv_bridge
        rclcpp
        rclcpp_components
        class_loader
        std_msgs
        image_transport
        geometry_msgs
        )
# コンポーネントノードをamentのリソースインデクスに登録する
rclcpp_components_register_nodes(autonomous_component "autonomous::Autonomous")


include_directories(
        ${sensor_msgs_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${rclcpp_components_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${image_transport_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
)


# add_library(autorace src/autorace.cpp include/signal_lib/xcalc_hog.c include/signal_lib/xcalc_hog_linux.c  include/signal_lib/xcalc_hog_sinit.c include/signal_lib/linuxmisc.cpp include/signal_lib/linuxmmap.cpp include/json_lib/json11.cpp)
#add_definitions(-std=c++11)

# スタンドアローンノードの実行ファイルをコンパイルする
add_executable(autonomous src/autonomous.cpp)
# 共有ライブラリに実行ファイルをリンクする
target_link_libraries(autonomous autonomous_component)

# 実行ファイルのコンパイルターゲットに依存パッケージの情報を追加する
ament_target_dependencies(autonomous
        sensor_msgs
        cv_bridge
        rclcpp
        rclcpp_components
        std_msgs
        class_loader
        image_transport
        geometry_msgs
        )

# 本パッケージが利用するパッケージの情報を他のパッケージにエクスポートする
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(class_loader)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(image_transport)
ament_export_dependencies(geometry_msgs)

ament_export_include_directories(${sensor_msgs_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${rclcpp_components_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${image_transport_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
        )

ament_export_libraries(autonomous_component)

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