#include <cmath>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void display_cube(ros::Publisher &marker_pub, double x, double y, double theta) {
    uint32_t shape = visualization_msgs::Marker::CUBE;
    double side = .5;
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
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = side / 2;

    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, theta);

    marker.pose.orientation.x = orientation.getX();
    marker.pose.orientation.y = orientation.getY();
    marker.pose.orientation.z = orientation.getZ();
    marker.pose.orientation.w = orientation.getW();

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = side;
    marker.scale.y = side;
    marker.scale.z = side;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            ROS_ERROR("Error while waiting for a subscriber to the marker");
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub.publish(marker);
}

void clear_cube(ros::Publisher & marker_pub) {
    uint32_t shape = visualization_msgs::Marker::CUBE;
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
    marker.action = visualization_msgs::Marker::DELETE;

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            ROS_ERROR("Error while waiting for a subscriber to the marker");
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub.publish(marker);
}

void wait_for_robot_at(ros::NodeHandle n, double target_x, double target_y, double tolerance) {
    const double tolerance_square = tolerance*tolerance;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(5);
    while (n.ok()) {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("map", "base_footprint",
                                                        ros::Time(0));
            double x = transformStamped.transform.translation.x;
            double y = transformStamped.transform.translation.y;
            ROS_INFO("x=%f y=%f", x, y);
            if (pow(x-target_x, 2)+pow(y-target_y, 2) <= tolerance_square)
                break;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    double tolerance = .5;
    double start_x = 0;
    double start_y = -4;
    double finish_x = 0;
    double finish_y = 4;

    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    display_cube(marker_pub, start_x, start_y, 0);
    wait_for_robot_at(n, start_x, start_y, tolerance);
    clear_cube(marker_pub);
    ros::Duration(5.0).sleep();
    wait_for_robot_at(n, finish_x, finish_y, tolerance);
    display_cube(marker_pub, finish_x, finish_y, 0);
}