# arav-air
ARAV is an Autonomous Robotic Aerial Vehicle developed by Alberto Ceballos and Joan Bessa at ISAE-SUPAERO.

## Required software and libraries

Read carefully the installation instructions before cloning this repository, additional software and libraries are required.

## Clone the repository

Git submodules are used in this project. Please use the recursive flag when cloning the repository:

`git clone --recursive <Github Link>`

This process may take a long time since several submodules are downloaded as part of the source code.

## Compile the project

Compile both the project source code and the PX4 Autopilot firmware:

`cd arav_air`

`catkin_make`

`cd px4`

`make px4_sitl_default gazebo`

This process may take a long time too. When the PX4 Autopilot firmware is compiled, a Gazebo GUI window is automatically opened. Please enable the offboard mode with the following command (paste it in the command line):

`param set COM_RCL_EXCEPT 4`

Then close Gazebo using the Ctrl-C command.

## Download object detection model and weights

Go back to the project base directory and download the detection model from Google drive:

`cd ..`

`cd src/perception/model/variables`

`./model.sh`

## Copy Gazebo models to Gazebo default directory

Go back to the project base directory and move the models to the Gazebo default directory:

`cd ../../..`

`cd simulation/models/`

`./copy.sh`

## Launch a demo of the project

Close current command line window and open a new one in the base directory, then execute the demo:

`cd arav_air`

`./demo.sh`

Enjoy the simulation !!
