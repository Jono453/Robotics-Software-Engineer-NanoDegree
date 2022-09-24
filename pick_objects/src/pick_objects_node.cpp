#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


//TODO include extra goal position and orientation for robot to reach
// First Goal = desired pickup goal 
// Second Goal = desired dro[ off goal
// Go to pickup goal - display message it reached pick up - wait 5 sec - travel to drop off goal - display message it reached drop off goal

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goalPickup;
  move_base_msgs::MoveBaseGoal goalDropoff;

  // set up the frame parameters
  goalPickup.target_pose.header.frame_id = "map"; //changed from base_link to map
  goalPickup.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goalPickup.target_pose.pose.position.x = 0.0;
  goalPickup.target_pose.pose.position.y = -0.0;
  goalPickup.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goalPickup);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Pick Up Goal Position Reached");
  else
    ROS_INFO("Pick Up Goal Failure");

  return 0;
}
