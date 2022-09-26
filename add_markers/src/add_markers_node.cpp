#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

/*
First Goal = desired pickup goal (1.0,6.5)
Second Goal = desired drop off goal (-0.4,-4.8)
*/

//#define DEBUG

#define PICKUP_X_GOAL 1.0
#define PICKUP_Y_GOAL 6.5
#define DROPOFF_X_GOAL -0.4
#define DROPOFF_Y_GOAL -4.8

bool isAtPickup = false;
bool isAtDropOff = false;
bool restAtGoal = false;

void VelocityCallback(const geometry_msgs::Twist& msg)
{

	if (msg.linear.x <= 0.1 && msg.linear.y <= 0.1)
	{
		restAtGoal = true;
		//ROS_INFO("--ROBOT is at Rest in Goal!--");
	}
	else
	{
		restAtGoal = false;
	}
}

// This callback function continuously executes and reads the odometry from robot /amcl_pose topic
// Message header notes at http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovariance.html
void checkGoalProximity(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

	//ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);

	// Checking Euclidean distanc between current pose x and y and pick up/drop off goal positions x and y
	float pickUpThreshold = sqrt( abs((pow(msg->pose.pose.position.x,2) - pow(PICKUP_X_GOAL,2))) + abs((pow(msg->pose.pose.position.y,2) - pow(PICKUP_Y_GOAL,2))) );		
	float dropOffThreshold = sqrt( abs((pow(msg->pose.pose.position.x,2) - pow(DROPOFF_X_GOAL,2))) + abs((pow(msg->pose.pose.position.y,2) - pow(DROPOFF_Y_GOAL,2))) );

	#ifdef DEBUG	
		ROS_INFO("pickUpThreshold: [%f]",pickUpThreshold);
		ROS_INFO("dropOffThreshold: [%f]",dropOffThreshold);
	#endif

	if (pickUpThreshold <= 0.1 && isAtPickup == false && restAtGoal == true)
	{
		isAtPickup = true; //remains unchanged after robot has reached initial pick up goal
		restAtGoal = false; //reset
		ROS_INFO("Odometry -> At PICK UP GOAL");
	}
	else if (dropOffThreshold <= 0.1 && isAtDropOff == false && isAtPickup == true && restAtGoal == true)
	{
		isAtDropOff = true;
		restAtGoal = false; //reset
		ROS_INFO("Odometry -> At DROP OFF GOAL");
	}

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Subscribe to /amcl_pose topic to determine if the robot is near the pick up and drop off goal positions
  ros::Subscriber sub1 = n.subscribe("/amcl_pose", 100, checkGoalProximity);
  // Subscribe to /cmd_vel to assist with determining if robot is really at goal and ready to pick up or drop off marker
  ros::Subscriber sub2 = n.subscribe("/cmd_vel", 100, VelocityCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)

	// ----------------Marker initially present at the Pick Up Goal----------------//
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker at PICK UP Goal
    marker.pose.position.x = PICKUP_X_GOAL;
    marker.pose.position.y = PICKUP_Y_GOAL;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    //marker.lifetime = ros::Duration(); 

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

	// ----------------Robot Reaches Pick Up Goal----------------//
	// Only display Marker when robot is near Pickup Goal
	// Make it disappear once robot has 'picked up' the marker
	if (isAtPickup == true)
	{
	    marker.action = visualization_msgs::Marker::DELETE;
		ROS_WARN_ONCE("Marker - [Deleted] Picked Up");
        marker_pub.publish(marker);
		ros::Duration(2.0).sleep();
	}

	// ----------------Robot Reaches Drop Off Goal----------------//
	// Display Marker again ONLY when robot is near Drop Off Goal
	// Make it re-appear once robot has 'dropped off" the marker
	if (isAtDropOff == true && isAtPickup == true)
	{
		marker.action = visualization_msgs::Marker::ADD;
    	marker.pose.position.x = DROPOFF_X_GOAL;
	    marker.pose.position.y = DROPOFF_Y_GOAL;
		ROS_WARN_ONCE("Marker - [Added] Dropped Off");
		marker_pub.publish(marker);
		ROS_INFO("Completed Home Service Simulation");
		ros::Duration(5.0).sleep();
		return 0;
	}	

	if (isAtDropOff == false && isAtPickup == false)
	{	
		marker_pub.publish(marker); //publish marker at pick up goal until robot arrives there
	}

    // Handle ROS communication events
    ros::spinOnce();    
	
	r.sleep();
  }
}
