cmake_minimum_required(VERSION 3.8)
project(samxl_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tesseract_rosutils REQUIRED)
find_package(tesseract_monitoring REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Build an executable from the planning server code
add_executable(${PROJECT_NAME} src/planning_server.cpp)
ament_target_dependencies(${PROJECT_NAME}
  tesseract_rosutils
  tesseract_monitoring
  std_srvs
)

# Install the launch file directory
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install the executable(s)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
