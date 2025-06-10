cmake_minimum_required(VERSION 3.5)
project(mycontrol)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_msg REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(mycontrol src/mycontrol.cpp)
ament_target_dependencies(mycontrol
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "std_msgs"
  "my_msg"
  "nav_msgs"
)

target_link_libraries(mycontrol i2c)

# Install the executable
install(TARGETS
  mycontrol
  DESTINATION lib/${PROJECT_NAME}
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

