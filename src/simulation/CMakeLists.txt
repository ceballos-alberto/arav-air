# ---------------------------------------------------------------------------- #
# ---------------------- Self-Driving Simulator - ARAV ----------------------- #
# ---------------------------------------------------------------------------- #
# -------------------- Author : Alberto Ceballos Gonzalez -------------------- #
# -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- #
# --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved -------- #
# ---------------------------------------------------------------------------- #

cmake_minimum_required (VERSION 3.0.2)
project (simulation)
find_package (
  catkin REQUIRED
  rviz
  sensor_msgs
  std_msgs
  cv_bridge
  rospy 
)
find_package (OpenCV REQUIRED)
include_directories (include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# ---------------------------------------------------------------------------- #
# ---------------------- Self-Driving Simulator - ARAV ----------------------- #
# ---------------------------------------------------------------------------- #
# -------------------- Author : Alberto Ceballos Gonzalez -------------------- #
# -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- #
# --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved -------- #
# ---------------------------------------------------------------------------- #
