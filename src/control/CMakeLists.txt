# ---------------------------------------------------------------------------- #
# -------------------- Aerial Control Module (ACM) - ARAV -------------------- #
# ---------------------------------------------------------------------------- #
# -------------------- Author : Alberto Ceballos Gonzalez -------------------- #
# -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- #
# --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved -------- #
# ---------------------------------------------------------------------------- #

cmake_minimum_required (VERSION 3.0.2)
project (control)
find_package (
  catkin REQUIRED
  std_msgs
  geometry_msgs
  mavros_msgs
  roscpp 
)
SET (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/code)				
include_directories (include ${catkin_INCLUDE_DIRS})
add_executable (px4pub code/px4pub.cpp)
target_link_libraries (px4pub ${catkin_LIBRARIES})

# ---------------------------------------------------------------------------- #
# -------------------- Aerial Control Module (ACM) - ARAV -------------------- #
# ---------------------------------------------------------------------------- #
# -------------------- Author : Alberto Ceballos Gonzalez -------------------- #
# -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- #
# --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved -------- #
# ---------------------------------------------------------------------------- #

