cmake_minimum_required(VERSION 3.5)
project(autonomous_composition)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autonomous REQUIRED)
find_package(pcam REQUIRED)
find_package(webcam REQUIRED)
find_package(red_detect REQUIRED)

# 他のノードを統合するノードをコンパイルする
add_executable(autonomous_composition src/autonomous_composition.cpp)
ament_target_dependencies(autonomous_composition
  rclcpp
  autonomous
  pcam
  webcam
  red_detect
  )

install(TARGETS
        autonomous_composition
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()
