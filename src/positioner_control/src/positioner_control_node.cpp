
#include "ros/ros.h"
#include <ros/console.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <sstream>

const double tau = 2 * M_PI;

void log_pose(geometry_msgs::Pose pose)
{
    ROS_INFO_NAMED("current state","Position:(%f,%f,%f), orientation (%f,%f,%f %f) ",
                   pose.position.x,pose.position.y,pose.position.z,
                   pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "positioner_control_node");

    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "rotation_group";

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    const moveit::core::JointModelGroup* joint_model_group =
            move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    std::vector<std::string> joint_names = joint_model_group->getJointModelNames();

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

    visual_tools.loadRemoteControl();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    visual_tools.prompt("Press 'next' to plan the motion");
    geometry_msgs::Pose start_pose = move_group_interface.getCurrentPose().pose;
    log_pose(start_pose);

    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    std::vector<double> joint_group_positions;

    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] -= tau / 4;
    move_group_interface.setJointValueTarget(joint_group_positions);

    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);

    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("ros_control_node", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui to execute motion");

    move_group_interface.execute(my_plan);
    geometry_msgs::Pose end_pose = move_group_interface.getCurrentPose().pose;
    log_pose(end_pose);

    return 0;
}
