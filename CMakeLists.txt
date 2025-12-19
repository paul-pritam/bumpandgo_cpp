cmake_minimum_required(VERSION 3.8)
project(fsm_bumpgo_cpp)

# Default to C++ standard from the compiler
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/fsm_bumpgo_cpp/bumpgonode.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(bumpgo src/bumpgo_main.cpp)
ament_target_dependencies(bumpgo ${dependencies})
target_link_libraries(bumpgo ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  bumpgo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
