#!/usr/bin/env python3

""" ----------------------------------------------------------------------------
    ------------------------- Waypoint Publisher - ARAV ------------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez --------------------
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------
    ---------------------------------------------------------------------------- """

# Import required libraries #

import rospy
from std_msgs.msg import Float64MultiArray

# Topic names #

waypointTopic = "/arav/path/waypoints"

# WayPoints #

waypointList = [

[0.0,0.00,1.5],[1.0,0.00,1.75],[2.0,0.0,2.0],[3.0,0.0,2.25],[4.0,0.00,2.5],[5.0,0.0,2.75],
[6.0,0.0,2.75],[7.0,0.0,2.50],[8.0,0.0,2.25],[9.0,0.0,2.0],[10.0,0.0,1.75],[11.0,0.0,1.5]

]

# Init node #

rospy.init_node("WaypointPublisher")

# Loop rate (Hz) #

updateRate = 2
rate = rospy.Rate(updateRate)

# Publishers #

waypointPub = rospy.Publisher (waypointTopic, Float64MultiArray, queue_size=50)

outputMsg = Float64MultiArray ()

# Variables #

iterator = -1
waypoint = [0.0,0.0,0.0]

# Node Main Loop #

while not rospy.is_shutdown() and iterator<len(waypointList):

	waypoint[0] = waypointList[iterator][0]
	waypoint[1] = waypointList[iterator][1]
	waypoint[2] = waypointList[iterator][2]

	# Publish message #

	outputMsg.data = waypoint

	waypointPub.publish(outputMsg)

	rate.sleep ()

	iterator += 1

# End of the Loop #

""" ----------------------------------------------------------------------------
    ------------------------- Waypoint Publisher - ARAV ------------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez --------------------
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------
    ---------------------------------------------------------------------------- """
