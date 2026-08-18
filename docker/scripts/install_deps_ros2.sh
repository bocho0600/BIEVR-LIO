#!/bin/bash
set -o pipefail

# Install system deps from apt
apt-get -qq update &&  apt-get install -y libgoogle-glog-dev git

# grid_map deps that jazzy-desktop-full does not already ship: grid_map_cv pulls
# in `filters` and grid_map_ros pulls in `nav2_msgs`. The ROS1 image gets both of
# its equivalents from noetic-desktop-full, so install_deps_ros1.sh needs nothing.
apt-get install -y ros-jazzy-filters ros-jazzy-nav2-msgs

# Clear cache to keep layer size down
rm -rf /var/lib/apt/lists/*
