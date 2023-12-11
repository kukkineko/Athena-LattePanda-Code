cmake_minimum_required(VERSION 3.5)
project(athena-self-driving)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ... any other packages needed ...

# Add executable
add_executable(cppclient src/CppClient.cpp)
# Replace 'src/CppClient.cpp' with the path to your source file within the package

# Specify libraries to link a target against
ament_target_dependencies(cppclient
  rclcpp
  sensor_msgs
  # ... other dependencies ...
)

install(TARGETS
  cppclient
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
