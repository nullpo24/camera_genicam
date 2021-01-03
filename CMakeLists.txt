cmake_minimum_required(VERSION 3.5)
project(camera_genicam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(RC_GENICAM_API REQUIRED)
find_package(GLIB2 REQUIRED)

include_directories(${RC_GENICAM_API_INCLUDE_PATH} 
                    /usr/include/rc_genicam_api/genicam/
                    ${GLIB2_INCLUDE_DIRS}
                    )

add_executable(camnode src/camnode.cpp)
ament_target_dependencies(camnode rclcpp std_msgs sensor_msgs)

install(TARGETS
  camnode
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(camnode ${RC_GENICAM_API_LIBRARY})
ament_package()
