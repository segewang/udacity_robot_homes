#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <add_markers/DealMarkerAt.h>

ros::ServiceClient client;

void dealMarkerAt(move_base_msgs::MoveBaseGoal goal, float status)
{
    add_markers::DealMarkerAt srv;
    srv.request.x = goal.target_pose.pose.position.x;
    srv.request.y = goal.target_pose.pose.position.y;
    srv.request.status = status;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service dealMarkerAt");
}

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;

  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<add_markers::DealMarkerAt>("/add_markers/deal_marker_at");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal_pickup;

  // set up the frame parameters
  goal_pickup.target_pose.header.frame_id = "map";
  goal_pickup.target_pose.header.stamp = ros::Time::now();

  // Define the pickup position and orientation for the robot to reach
  goal_pickup.target_pose.pose.position.x = -1.0;
  goal_pickup.target_pose.pose.position.y = 0.0;
  goal_pickup.target_pose.pose.orientation.z = 1.0;

  // Send the pickup goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup goal");
  ac.sendGoal(goal_pickup);
  ROS_INFO("Adding pickup marker");
  dealMarkerAt(goal_pickup, 1);

  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("OK! Reach the pickup goal");
    dealMarkerAt(goal_pickup, 0);
    ROS_INFO("Deleting pickup marker");
  }
  else
  {
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  }
  // Wait 5 sec 
  ROS_INFO("Waiting 5 sec");
  ros::Duration(5).sleep();

  move_base_msgs::MoveBaseGoal goal_dropoff;
  // set up the frame parameters
  goal_dropoff.target_pose.header.frame_id = "map";
  goal_dropoff.target_pose.header.stamp = ros::Time::now();
  // Define the dropoff position and orientation for the robot to reach
  goal_dropoff.target_pose.pose.position.x = -3.0;
  goal_dropoff.target_pose.pose.position.y = -1.0;
  goal_dropoff.target_pose.pose.orientation.z = 1.0;

  // Send the dropoff goal position and orientation for the robot to reach
  ROS_INFO("Sending dropoff goal");
  ac.sendGoal(goal_dropoff);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("OK! Reach the dropoff goal");
    ROS_INFO("Adding dropoff marker");
    dealMarkerAt(goal_dropoff, 1);
  }
  else
  {
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  }

  return 0;
}
