#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <add_markers/DealMarkerAt.h>

ros::Publisher marker_pub;

bool handle_add_markers_request(add_markers::DealMarkerAt::Request& req, add_markers::DealMarkerAt::Response& res)
{
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    ROS_INFO("DealMarkerAtRequest received - x:%1.2f, y:%1.2f, status:%1.2f", (float)req.x, (float)req.y, (float)req.status);

    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    std::string ad= "";
    if(req.status > 0)
    {
        marker.action = visualization_msgs::Marker::ADD;
        ad = "Add";
    }
    else
    {
        marker.action = visualization_msgs::Marker::DELETE;
        ad = "Delete";
    }
    
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = req.x;
    marker.pose.position.y = req.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);

    // Return a response message
    res.msg_feedback = ad + " Marker - x: " + std::to_string(req.x) + " , y: " + std::to_string(req.y);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::ServiceServer service = n.advertiseService("/add_markers/deal_marker_at", handle_add_markers_request);
  ROS_INFO("Ready to deal marker");

  ros::spin();
}
