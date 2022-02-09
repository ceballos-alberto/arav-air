/* ----------------------------------------------------------------------------
   -------------------- Flight Control Module (FCM) - ARAV --------------------
   ----------------------------------------------------------------------------
   -------------------- Author : Alberto Ceballos Gonzalez --------------------
   -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
   --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved --------
   ---------------------------------------------------------------------------- */

/* Import required libraries */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <cmath>
#include <string>
#include <unistd.h>

/* Namespaces */

using namespace std;
using namespace ros;
using namespace geometry_msgs;
using namespace mavros_msgs;
using namespace std_msgs;

/* Topic names - Inputs */

static const std::string STATE_TOPIC = "mavros/state";
static const std::string WAYPOINT_TOPIC = "arav/path_planning/output/path_aerial";
static const std::string ACTIVE_TOPIC = "arav/path_selector/aerial_activation";
static const std::string POSE_TOPIC = "mavros/local_position/pose";

/* Topic names - Outputs & services */

static const std::string CMD_TOPIC = "mavros/setpoint_position/local";
static const std::string ARM_TOPIC = "mavros/cmd/arming";
static const std::string MODE_TOPIC = "mavros/set_mode";

/* Shared variables for Callbacks */

mavros_msgs::State current_state;
std::vector<std::vector<double>> waypointList;
bool activation = false;
double current_position [3];

/* Callbacks */

void state_callback (const mavros_msgs::State::ConstPtr& msg) {
  current_state = *msg;
}

void waypoint_callback (const std_msgs::Float64MultiArray::ConstPtr& msg) {
  std::vector<double> waypoint = msg -> data;
  waypoint[0] = -waypoint[0];
  waypointList.push_back (waypoint);
}

void activation_callback (const std_msgs::Bool::ConstPtr& msg) {
  activation = msg -> data;
}

void position_callback (const geometry_msgs::PoseStamped::ConstPtr& msg) {
  current_position[0] = msg -> pose.position.x;
  current_position[1] = msg -> pose.position.y;
  current_position[2] = msg -> pose.position.z;
}

/* Important functions - Waypoint selector */

std::vector<double> waypointSelector (float margin) {

  std::vector<double> nextWaypoint (3,-1);

  if (waypointList.size() != 0) {

    /* There are waypoints in the list */

    float deltaX = std::abs (current_position[0] - waypointList[0][0]);
    float deltaY = std::abs (current_position[1] - waypointList[0][1]);
    float deltaZ = std::abs (current_position[2] - waypointList[0][2]);

    nextWaypoint[0] = waypointList[0][0];
    nextWaypoint[1] = waypointList[0][1];
    nextWaypoint[2] = waypointList[0][2];

    if (deltaX < margin && deltaY < margin && deltaZ < margin) {

      /* The waypoint has been reached */

      if (waypointList.size() == 1) {

        /* Last waypoint */

        waypointList.erase(waypointList.begin());

      }

      else {

        /* No last waypoint */

        waypointList.erase(waypointList.begin());

        nextWaypoint[0] = waypointList[0][0];
        nextWaypoint[1] = waypointList[0][1];
        nextWaypoint[2] = waypointList[0][2];

      }

    }

    return nextWaypoint;

  }

  /* There are no waypoints in the list */

  return nextWaypoint;

}

/* Main function */

int main(int argc, char **argv) {

  /* Script variables */

  std::string MARGIN_TAG (argv[1]);
  float margin = std::stof (MARGIN_TAG);

  std::string RATE_TAG (argv[2]);
  float rate_value = std::stof (RATE_TAG);

  std::string LEVEL_TAG (argv[3]);
  float initialLevel = std::stof (LEVEL_TAG);

  /* Init ROS node */

  ros::init (argc, argv, "aircontrol");
  ros::NodeHandle nh;

  /* Publishers, Subscribers & Services */

  ros::Subscriber state_sub = nh.subscribe <mavros_msgs::State> (STATE_TOPIC, 10, state_callback);
  ros::Subscriber waypoint_sub = nh.subscribe <std_msgs::Float64MultiArray> (WAYPOINT_TOPIC, 10, waypoint_callback);
  ros::Subscriber active_sub = nh.subscribe <std_msgs::Bool> (ACTIVE_TOPIC, 10, activation_callback);
  ros::Subscriber pose_sub = nh.subscribe <geometry_msgs::PoseStamped> (POSE_TOPIC, 10, position_callback);
  ros::Publisher cmd_pub = nh.advertise <geometry_msgs::PoseStamped> (CMD_TOPIC, 10);
  ros::ServiceClient arming_service = nh.serviceClient <mavros_msgs::CommandBool> (ARM_TOPIC);
  ros::ServiceClient mode_service = nh.serviceClient <mavros_msgs::SetMode> (MODE_TOPIC);

  /* Update rate */

  ros::Rate rate(rate_value);

  /* Wait until mavros is ready */

  while (ros::ok() && !current_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }

  /* Wait activation message */

  while (ros::ok() && activation == false) {
    ros::spinOnce();
    rate.sleep();
  }

  /* Initial command */

  std::vector<double> nextWaypoint;
  geometry_msgs::PoseStamped command;
  command.pose.position.x = 0.0;
  command.pose.position.y = 0.0;
  command.pose.position.z = initialLevel;

  /* Send a few setpoints before starting */

  for (int i = 100; ros::ok() && i > 0; --i) {
    cmd_pub.publish(command);
    ros::spinOnce();
    rate.sleep();
  }

  /* Arm UAV >> Ready for flight */

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  /* Main loop */

  bool status = false;
  int counter = 0;
  ros::Time last_request = ros::Time::now();

  while (ros::ok()) {

    /* Enable Offboard mode */

    if (current_state.mode!="OFFBOARD"&&(ros::Time::now()-last_request>ros::Duration(5.0))) {
      if(mode_service.call(offb_set_mode)&&offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    }

    /* Arm UAV */

    else {
      if(!current_state.armed&&(ros::Time::now()-last_request>ros::Duration(5.0))) {
        if(arming_service.call(arm_cmd)&&arm_cmd.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    /* Hovering at initial point */

    if (counter < 500) {
      counter++;
      command.pose.position.x = 0.0;
      command.pose.position.y = 0.0;
      command.pose.position.z = initialLevel;
      cmd_pub.publish(command);
    }

    /* UAV Flight */

    else {

      /* Select following waypoint */

      nextWaypoint = waypointSelector (margin);

      if (nextWaypoint[0] == -1.0) {

        /* The list of waypoints is empty */

        if (status == true) {
          /* Last waypoint reached */
          break;
        }

        /* Waypoints not reveived yet */

        command.pose.position.x = 0.0;
        command.pose.position.y = 0.0;
        command.pose.position.z = initialLevel;
        cmd_pub.publish(command);

      }

      else {

        /* Go to the following waypoint */

        status = true;

        command.pose.position.x = nextWaypoint[0];
        command.pose.position.y = nextWaypoint[1];
        command.pose.position.z = nextWaypoint[2];
        cmd_pub.publish(command);

      }

    }

    ros::spinOnce();
    rate.sleep();

  }

  /* Hovering at final point */

  for (int i = 100; ros::ok() && i > 0; --i) {
    cmd_pub.publish(command);
    ros::spinOnce();
    rate.sleep();
  }

  /* Flight has finished >> Land */

  offb_set_mode.request.custom_mode = "AUTO.LAND";
  last_request = ros::Time::now();

  while (ros::ok()) {

    /* Enable land mode */

    if(current_state.mode!="AUTO.LAND"&&(ros::Time::now()-last_request>ros::Duration(5.0))) {
      if(mode_service.call(offb_set_mode)&&offb_set_mode.response.mode_sent) {
        ROS_INFO("Landing enabled");
        break;
      }
      last_request = ros::Time::now();
    }

    cmd_pub.publish(command);
    ros::spinOnce();
    rate.sleep();

  }

  /* Send a few setpoints after finishing */

  for (int i = 200; ros::ok() && i > 0; --i) {
    cmd_pub.publish(command);
    ros::spinOnce();
    rate.sleep();
  }

  /* Wait and shutdown ROS node */

  sleep(7);

  /* Exit main function */

  return 0;

}

/* ----------------------------------------------------------------------------
   -------------------- Flight Control Module (FCM) - ARAV --------------------
   ----------------------------------------------------------------------------
   -------------------- Author : Alberto Ceballos Gonzalez --------------------
   -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
   --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved --------
   ---------------------------------------------------------------------------- */
