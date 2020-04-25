#include <cmath>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * Display a cube marker with a given 2D pose, expressed in the map frame.
 * @param marker_pub A publisher for topic `visualization_marker` that will be used to request creation of the marker.
 * @param x The x coordinate for the marker pose.
 * @param y The y coordinate for the marker pose.
 * @param theta The yaw coordinate for the marker pose.
 */
void display_cube(ros::Publisher &marker_pub, double x, double y, double theta) {
    double side = .5; // Side of the marker cube, in meters
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // It can be used for subsequent deletion of the marker
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header.
    // Components that are not set default to 0
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = side / 2; // The cube must rest on the floor

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

/** Remove the marker cube
 * A publisher for topic `visualization_marker` that will be used to request deletion of the marker.
 * @param marker_pub
 */
void clear_cube(ros::Publisher & marker_pub) {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for the marker to be deleted.
    marker.ns = "add_markers";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action to DELETE
    marker.action = visualization_msgs::Marker::DELETE;

    // Publish the marker deletion
    while (marker_pub.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            ROS_ERROR("Error while waiting for a subscriber to the marker");
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub.publish(marker);
}

/** Waits for the origin of frame base_footprint (which essentially tracks the pose of the robot) to be close enough to
 * the given 2D coordinates in the map frame (within a tolerance), then returns. It ignores the robot orientation.
 * @param node the add_markers ROS node.
 * @param target_x the target x coordinates.
 * @param target_y the target y coordinates.
 * @param tolerance the maximum distance from the base_footprint frame origin and the given coordinates that is
 * considered close enough.
 */
void wait_for_robot_at(ros::NodeHandle node, double target_x, double target_y, double tolerance) {
    const double tolerance_square = tolerance*tolerance;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(4);
    /* Wait for the origin of frame base_footprint (which tracks the pose of the robot) to be close enough to
     * the given target coordinates; i.e. at a distance that is less than the given tolerance */
    while (node.ok()) {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("map", "base_footprint",
                                                        ros::Time(0));
            double x = transformStamped.transform.translation.x;
            double y = transformStamped.transform.translation.y;
            // ROS_INFO("x=%f y=%f", x, y);
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
    ros::NodeHandle node;

    /* Coordinates (in map frame) for the pickup and drop-off locations, and tolerance by which the robot must reach
     * those coordinates, in meters */
    double tolerance = .33;
    double pickup_x = -5;
    double pickup_y = 4;
    double dropoff_x = 5;
    double dropoff_y = -5;

    ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    display_cube(marker_pub, pickup_x, pickup_y, 0);
    wait_for_robot_at(node, pickup_x, pickup_y, tolerance);
    clear_cube(marker_pub);
    ros::Duration(5.0).sleep();
    wait_for_robot_at(node, dropoff_x, dropoff_y, tolerance);
    display_cube(marker_pub, dropoff_x, dropoff_y, 0);
}