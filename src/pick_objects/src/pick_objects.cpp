#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool send_robot(double x, double y, double theta, MoveBaseClient &ac) {

    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, theta);
    goal.target_pose.pose.orientation.w = orientation.getW();
    goal.target_pose.pose.orientation.x = orientation.getX();
    goal.target_pose.pose.orientation.y = orientation.getY();
    goal.target_pose.pose.orientation.z = orientation.getZ();


    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal (x, y, theta)=(%f, %f, %f).", x, y, theta);
    ac.sendGoal(goal);

    // Wait an unlimited time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("The base moved to the designated goal.");
        return true;
    } else {
        ROS_INFO("The base failed to reach the designated goal");
        return false;
    }

}

int main(int argc, char **argv) {
    double start_x = -5;
    double start_y = 4;
    double finish_x = 5;
    double finish_y = -5;
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    bool result = send_robot(start_x, start_y, .0, ac);
    ros::Duration(5).sleep();
    if (result)
        send_robot(finish_x, finish_y, .0, ac);

    return 0;
}