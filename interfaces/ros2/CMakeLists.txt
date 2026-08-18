cmake_minimum_required(VERSION 3.16)
project(bievr_lio_ros2)

# Declare the C++ standard via CMake (not a raw -std flag): bievr_lio links Ceres
# which propagates a cxx_std_17 compile feature, and a raw -std=c++2a would be
# overridden by the -std=gnu++17 CMake appends for that feature. CMAKE_CXX_STANDARD
# makes CMake take the max (C++20) and picks the right spelling per compiler
# (-std=c++2a on GCC<10, -std=c++20 on GCC>=10). GCC still needs -fconcepts.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-fconcepts)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(bievr_lio REQUIRED)         # plain-CMake package: provides bievr_lio::bievr_lio
find_package(bievr_ros_common REQUIRED)  # plain-CMake package: shared header-only glue
find_package(TBB REQUIRED)
find_package(yaml-cpp REQUIRED)    # shared config_loader.h parses plain YAML files

# Livox is optional: only livox_ros_driver2 exists for ROS2.
find_package(livox_ros_driver2 QUIET)

# grid_map is optional: the 2.5D elevation map publishing is only built when all
# three grid_map packages are available.
find_package(grid_map_core QUIET)
find_package(grid_map_ros QUIET)
find_package(grid_map_msgs QUIET)

set(ament_dependencies
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  rosbag2_cpp
  rosbag2_storage
)
if(livox_ros_driver2_FOUND)
  list(APPEND ament_dependencies livox_ros_driver2)
  message(STATUS "bievr_lio_ros2: livox_ros_driver2 found, enabling Livox support.")
else()
  message(STATUS "bievr_lio_ros2: livox_ros_driver2 NOT found, building without Livox support.")
endif()

if(grid_map_core_FOUND AND grid_map_ros_FOUND AND grid_map_msgs_FOUND)
  set(BIEVR_GRID_MAP_FOUND TRUE)
  list(APPEND ament_dependencies grid_map_core grid_map_ros grid_map_msgs)
  message(STATUS "bievr_lio_ros2: grid_map found, enabling elevation map support.")
else()
  set(BIEVR_GRID_MAP_FOUND FALSE)
  message(STATUS "bievr_lio_ros2: grid_map NOT found, building without elevation map support.")
endif()

# ---- Executables ----
# The wrapper is now header-only apart from the two executables (conversions,
# publisher and config loading all live in headers shared via bievr_ros_common),
# so there is no library to build -- the executables link the dependencies
# directly.
add_executable(process_topics src/process_topics.cpp)
add_executable(process_bag src/process_bag.cpp)

foreach(target process_topics process_bag)
  ament_target_dependencies(${target} ${ament_dependencies})
  target_include_directories(${target} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${YAML_CPP_INCLUDE_DIR}
  )
  target_link_libraries(${target}
    bievr_lio::bievr_lio
    bievr_ros_common::bievr_ros_common
    TBB::tbb
    ${YAML_CPP_LIBRARIES}
  )
  if(livox_ros_driver2_FOUND)
    target_compile_definitions(${target} PRIVATE BIEVR_WITH_LIVOX)
  endif()
  if(BIEVR_GRID_MAP_FOUND)
    target_compile_definitions(${target} PRIVATE BIEVR_WITH_GRID_MAP)
  endif()
endforeach()

# ---- Install ----
install(TARGETS process_topics process_bag
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)
# Install the shared canonical YAMLs (BIEVR-LIO/config, a sibling of interfaces/)
# into this package's share so FindPackageShare(pkg)/config resolves at runtime.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${ament_dependencies})

ament_package()
