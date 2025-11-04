cmake_minimum_required(VERSION 3.8)
project(localization_switcher)

# warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# core library (component, graph, decider, node helpers)
add_library(localization_switcher_library
  src/localization_switcher_component.cpp
  src/internal/graph_builder.cpp
  src/internal/transition_decider.cpp
  src/lib/graph.cpp
  src/lib/node.cpp
  src/localization_switcher_node.cpp
)

ament_target_dependencies(localization_switcher_library
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)
target_link_libraries(localization_switcher_library
  yaml-cpp
)
target_include_directories(localization_switcher_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(localization_switcher_library PUBLIC cxx_std_17)

# node entry (main)
add_executable(localization_switcher_node_executable
  src/localization_switcher.cpp
)
# Use PLAIN signature to avoid mixing with ament's internal calls
target_link_libraries(localization_switcher_node_executable
  localization_switcher_library
)
ament_target_dependencies(localization_switcher_node_executable
  rclcpp
)

# install
install(TARGETS
  localization_switcher_library
  localization_switcher_node_executable
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

# (optional) install configs
# install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME})

# lint / tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
