cmake_minimum_required(VERSION 3.5)
project(autonomous)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(class_loader REQUIRED)

include_directories(include)

add_library(autonomous_component SHARED src/autonomous_component.cpp)
target_compile_definitions(autonomous_component PRIVATE "autonomous_BUILDING_DLL")
ament_target_dependencies(autonomous_component
        rclcpp
        class_loader
        )
rclcpp_register_node_plugins(autonomous_component "autonomous::autonomous")

add_executable(autonomous src/autonomous.cpp)
target_link_libraries(autonomous autonomous_component)
ament_target_dependencies(autonomous rclcpp)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(class_loader)

ament_export_include_directories(include)

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