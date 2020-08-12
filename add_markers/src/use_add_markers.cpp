#include <ros/ros.h>
#include <add_markers/DealMarkerAt.h>

ros::ServiceClient client;

void dealMarkerAt(float x, float y, float status)
{
    add_markers::DealMarkerAt srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.status = status;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service dealMarkerAt");
}

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  client = n.serviceClient<add_markers::DealMarkerAt>("/add_markers/deal_marker_at");

  // add marker
  dealMarkerAt(-1, 0, 1);

  // Wait 5 sec
  ROS_INFO("Waiting 5 sec");
  ros::Duration(5.0).sleep();

  // delete marker
  dealMarkerAt(-1, 0, 0);
  ROS_INFO("Sending pickup goal");

  // Wait 5 sec
  ROS_INFO("Waiting 5 sec");
  ros::Duration(5.0).sleep();

  // add marker
  ROS_INFO("Adding pickup marker");
  dealMarkerAt(-3, -1, 1);

  return 0;
}
