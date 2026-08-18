cmake_minimum_required(VERSION 3.16)
project(bievr_lio_ros LANGUAGES CXX)

# Declare the C++ standard via CMake's mechanism (not a raw -std flag): linking
# Ceres propagates a cxx_std_17 compile feature, and a raw -std=c++2a would be
# overridden by the -std=gnu++17 CMake appends for that feature. Setting the
# standard makes CMake take the max (C++20). GCC still needs -fconcepts.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fconcepts -fdiagnostics-color=always")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf
  tf_conversions
  rosbag
)
find_package(TBB REQUIRED)
find_package(yaml-cpp REQUIRED)  # shared config_loader.h parses plain YAML files
# bievr_lio (core) and bievr_ros_common (shared header-only glue) are plain-CMake
# packages (not catkin), so they are not pulled in via ${catkin_LIBRARIES}; find
# and link their exported targets explicitly.
find_package(bievr_lio REQUIRED)
find_package(bievr_ros_common REQUIRED)

# Livox support is optional: the Livox CustomMsg branches are only compiled when
# the corresponding driver is on the system. Users without the drivers can still
# build and run with sensor_msgs/PointCloud2 inputs.
#   livox_ros_driver  (gen1) -> BIEVR_WITH_LIVOX
#   livox_ros_driver2 (gen2) -> BIEVR_WITH_LIVOX2
find_package(livox_ros_driver QUIET)
find_package(livox_ros_driver2 QUIET)
if(livox_ros_driver_FOUND)
  message(STATUS "bievr_lio_ros: livox_ros_driver found, enabling Livox (gen1) support.")
else()
  message(STATUS "bievr_lio_ros: livox_ros_driver NOT found, building without Livox (gen1) support.")
endif()
if(livox_ros_driver2_FOUND)
  message(STATUS "bievr_lio_ros: livox_ros_driver2 found, enabling Livox (gen2) support.")
else()
  message(STATUS "bievr_lio_ros: livox_ros_driver2 NOT found, building without Livox (gen2) support.")
endif()

# grid_map is optional too: the 2.5D elevation map publishing is only built when
# both grid_map packages are available.
find_package(grid_map_core QUIET)
find_package(grid_map_ros QUIET)
if(grid_map_core_FOUND AND grid_map_ros_FOUND)
  set(BIEVR_GRID_MAP_FOUND TRUE)
  message(STATUS "bievr_lio_ros: grid_map found, enabling elevation map support.")
else()
  set(BIEVR_GRID_MAP_FOUND FALSE)
  message(STATUS "bievr_lio_ros: grid_map NOT found, building without elevation map support.")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    nav_msgs
    tf
    tf_conversions
    rosbag
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# The wrapper is now header-only apart from the two executables (conversions,
# publisher and config loading all live in headers shared via bievr_ros_common),
# so there is no library to build -- the executables link the dependencies
# directly.
add_executable(process_topics src/process_topics.cpp)
add_executable(process_bag src/process_bag.cpp)

foreach(target process_topics process_bag)
  # Ensure message headers from catkin dependencies exist first.
  add_dependencies(${target} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${target}
    ${catkin_LIBRARIES}
    bievr_lio::bievr_lio
    bievr_ros_common::bievr_ros_common
    ${YAML_CPP_LIBRARIES}
    TBB::tbb
  )
  # Optional Livox drivers: define the guard macros and pull in headers/libs
  # only when found (livox_ros_driver = gen1, livox_ros_driver2 = gen2).
  if(livox_ros_driver_FOUND)
    target_compile_definitions(${target} PRIVATE BIEVR_WITH_LIVOX)
    target_include_directories(${target} PRIVATE ${livox_ros_driver_INCLUDE_DIRS})
    target_link_libraries(${target} ${livox_ros_driver_LIBRARIES})
  endif()
  if(livox_ros_driver2_FOUND)
    target_compile_definitions(${target} PRIVATE BIEVR_WITH_LIVOX2)
    target_include_directories(${target} PRIVATE ${livox_ros_driver2_INCLUDE_DIRS})
    target_link_libraries(${target} ${livox_ros_driver2_LIBRARIES})
  endif()
  # Optional grid_map: elevation map publishing.
  if(BIEVR_GRID_MAP_FOUND)
    target_compile_definitions(${target} PRIVATE BIEVR_WITH_GRID_MAP)
    target_include_directories(${target} PRIVATE
      ${grid_map_core_INCLUDE_DIRS}
      ${grid_map_ros_INCLUDE_DIRS}
    )
    target_link_libraries(${target}
      ${grid_map_core_LIBRARIES}
      ${grid_map_ros_LIBRARIES}
    )
  endif()
endforeach()

install(TARGETS process_bag process_topics
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
