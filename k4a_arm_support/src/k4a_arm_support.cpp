#define _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS 1 // The C++ Standard doesn't provide equivalent non-deprecated functionality yet.
#pragma comment(lib, "windowsapp")

#include <ros/ros.h>
#include <cmath>
#include <memory>
#include <visualization_msgs/MarkerArray.h>
#include <vcruntime.h>
#include <windows.h>
#include <winrt/Windows.Foundation.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Int32.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>
#include <winml_msgs/DetectedObjectPose.h>
#include <marker_msgs/MarkerDetection.h>

using namespace std;
using namespace winrt;

ros::Publisher grip_pub;
ros::Subscriber gotoSub;
ros::Subscriber detectedObjectSub;
ros::Subscriber arucoDetectedObjectSub;
tf::TransformListener* listener;

moveit::planning_interface::MoveGroupInterface* move_group;
actionlib::SimpleActionClient<control_msgs::GripperCommandAction>* ac;

bool enablePlanning = true;

// Engine Block Cylindars
// x = 222mm, 1380mm
// y = 800mm
// z = 360mm, 750mm, 1140mm, 1530mm

double x1 = .0222;
double x2 = .1380;
double xEngineCenter = 0;
double yEngineCenter = 0;
double zEngineCenterFirst = -0.3;
double zEngineCenterSecond = .28;
double y = .08;

double correctionR = M_PI;
double correctionP = 0.0;
double correctionY = M_PI / 2.0;

double aruco_grip_end_offset = -0.045;
double place_end_offset = -0.005;

geometry_msgs::PoseStamped lastGripPose;
double zOffset = 0.28;

double moveit_group_planning_time = 10.0;
double moveit_group_goal_tolerance = 0.005;

geometry_msgs::PoseStamped lastPlacePose;

void openGripper()
{
  control_msgs::GripperCommandGoal goal;
  goal.command.position = 0.08;
  goal.command.max_effort = 30.0;
  ac->sendGoal(goal);
  ac->waitForResult(ros::Duration(10.0));
}

void closeGripper()
{
  control_msgs::GripperCommandGoal goal;
  goal.command.position = -0.017;
  goal.command.max_effort = 40.0;
  ac->sendGoal(goal);
  ac->waitForResult(ros::Duration(10.0));
}

void detectedObjectCallback(const winml_msgs::DetectedObjectPose::ConstPtr& msg)
{
    if (!listener->waitForTransform ("world", msg->header.frame_id, ros::Time(0), ros::Duration(.1)))
    {
        // not ready yet
        return;
    }

    tf::Transform enginePose;
    tf::poseMsgToTF(msg->pose, enginePose);

    tf::Vector3 gripOffset(xEngineCenter, yEngineCenter, zEngineCenterFirst);

    tf::StampedTransform engineToWorld;
    listener->lookupTransform("world", msg->header.frame_id, ros::Time(0), engineToWorld);

    auto engineGripPose = enginePose;

    engineGripPose.getOrigin() += gripOffset;
    engineGripPose = (tf::Transform)engineToWorld * engineGripPose;

    tf::Quaternion modelCorrection = tf::createQuaternionFromRPY(correctionR, correctionP, correctionY);
    auto gripRotationQ = engineGripPose * modelCorrection;
    gripRotationQ.normalize();
    engineGripPose.setRotation(gripRotationQ);

    tf::Stamped<tf::Pose> grasp_tf_pose(engineGripPose, ros::Time::now(), "world");
    {
        geometry_msgs::PoseStamped msg;
        tf::poseStampedTFToMsg(grasp_tf_pose, msg);

        lastGripPose = msg;
        zOffset = zEngineCenterSecond;
        grip_pub.publish(msg);
    }
}

void arucoDetectedObjectCallback(const marker_msgs::MarkerDetection::ConstPtr& msg)
{
    if (!listener->waitForTransform ("world", msg->header.frame_id, ros::Time(0), ros::Duration(.1)))
    {
        // not ready yet
        return;
    }

    if (0 == msg->markers.size())
    {
        return;
    }

    char buffer[50] = {};
    auto id = msg->markers[0].ids[0];
    sprintf(buffer, "t%d", id);

    if (!listener->waitForTransform ("world", buffer, ros::Time(0), ros::Duration(.1)))
    {
        // not ready yet
        ROS_WARN("cannot find world to %s tranform.", buffer);
        return;
    }

    tf::StampedTransform markerPose;
    listener->lookupTransform("world", buffer, ros::Time(0), markerPose);

    auto markerGripPose = markerPose;

    // ignore pitch & roll.
    double yaw, pitch, roll;
    markerGripPose.getBasis().getRPY(roll, pitch, yaw);
    markerGripPose.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));

    tf::Vector3 gripOffset(0.0, 0.0, 0.23);
    markerGripPose.getOrigin() += gripOffset;

    tf::Quaternion modelCorrection = tf::createQuaternionFromRPY(0.0, M_PI/2.0, 0.0);
    auto gripRotationQ = markerGripPose * modelCorrection;
    gripRotationQ.normalize();
    markerGripPose.setRotation(gripRotationQ);

    tf::Stamped<tf::Pose> grasp_tf_pose(markerGripPose, ros::Time::now(), "world");
    {
        geometry_msgs::PoseStamped msg;
        tf::poseStampedTFToMsg(grasp_tf_pose, msg);

        lastGripPose = msg;
        zOffset = aruco_grip_end_offset;
        grip_pub.publish(msg);
    }

    return;
}

void gotoCallback(const std_msgs::Int32::ConstPtr& msg)
{
    if (msg->data == 0)
    {
        openGripper();
        move_group->setNamedTarget("home");
        move_group->move();
        return;
    }
    else if (msg->data == 1)
    {
        ros::Duration diff = ros::Time::now() - lastGripPose.header.stamp;
        if (diff > ros::Duration(10.0))
        {
            ROS_INFO_NAMED("k4a", "ignore grip pose than 10s old");
            return;
        }

        move_group->setPlanningTime(moveit_group_planning_time);
        move_group->setPoseReferenceFrame("world");
        move_group->setGoalTolerance(moveit_group_goal_tolerance);

        move_group->stop();

        openGripper();
        move_group->setStartStateToCurrentState();
        move_group->setNamedTarget("home");
        move_group->move();

        ros::Duration(0.5).sleep();

        auto gripPoseMsg = lastGripPose;

        move_group->setStartStateToCurrentState();
        move_group->setPoseTarget(gripPoseMsg, "ee_link");

        moveit::planning_interface::MoveGroupInterface::Plan gripPlan;

        bool planned = false;
        planned = (move_group->plan(gripPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("k4a", "plan 1 (pose goal) %s", planned ? "" : "FAILED");

        if (!planned)
        {
            return;
        }

        ROS_INFO_STREAM("how many states: " << gripPlan.trajectory_.joint_trajectory.points.size());

        move_group->execute(gripPlan);

        gripPoseMsg.pose.position.z += zOffset;

        move_group->setStartStateToCurrentState();
        planned = move_group->setPoseTarget(gripPoseMsg, "ee_link");
        planned = (move_group->plan(gripPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("k4a", "grip (pose goal) %s", planned ? "" : "FAILED");

        if (planned)
        {
            move_group->execute(gripPlan);
            closeGripper();

            move_group->setStartStateToCurrentState();
            move_group->setNamedTarget("home");
            move_group->move();

            lastPlacePose = gripPoseMsg;
            lastPlacePose.header.stamp = ros::Time::now();
        }
    }
    else if (msg->data == 2)
    {
        ros::Duration diff = ros::Time::now() - lastPlacePose.header.stamp;
        if (diff > ros::Duration(30.0))
        {
            ROS_INFO_NAMED("k4a", "ignore place pose than 30s old");
            return;
        }

        auto placePose = lastPlacePose;

        move_group->setPlanningTime(moveit_group_planning_time);
        move_group->setPoseReferenceFrame("world");
        move_group->setGoalTolerance(moveit_group_goal_tolerance);

        move_group->stop();

        placePose.pose.position.z -= place_end_offset;
        move_group->setStartStateToCurrentState();
        move_group->setPoseTarget(placePose, "ee_link");

        moveit::planning_interface::MoveGroupInterface::Plan placePlan;

        bool planned = false;
        planned = (move_group->plan(placePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("k4a", "plan 1 (pose goal) %s", planned ? "" : "FAILED");

        if (!planned)
        {
            return;
        }

        move_group->execute(placePlan);
        openGripper();

        move_group->setStartStateToCurrentState();
        move_group->setNamedTarget("home");
        move_group->move();
        return;
    }
    else if (msg->data == 3)
    {
        // Auto Mode. Exercise for the reader
   
        move_group->setNamedTarget("home");
        move_group->move();
        return;
    }
}

int main(int argc, char **argv)
{
    /*
    while (!IsDebuggerPresent())
    {
        Sleep(5);
    }
    */

    winrt::init_apartment();
    ros::init(argc, argv, "k4a_arm_support");

    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    ros::AsyncSpinner async_spinner (3);
    async_spinner.start();

    std::string gripper_name;
    nhPrivate.param<std::string>("gripper_name", gripper_name, "gripper");
    nhPrivate.param<bool>("enablePlanning", enablePlanning, true);
    nhPrivate.param<double>("aruco_grip_end_offset", aruco_grip_end_offset, -0.045);
    nhPrivate.param<double>("planning_time", moveit_group_planning_time, 10.0);
    nhPrivate.param<double>("goal_tolerance", moveit_group_goal_tolerance, 0.005);
    nhPrivate.param<double>("place_end_offset", place_end_offset, -0.005);

    double planar_correct_roll = 0.0;
    double planar_correct_pitch = 0.0;
    double planar_correct_yaw = 0.0;
    nhPrivate.param<double>("planar_correct_roll", planar_correct_roll, 0.0);
    nhPrivate.param<double>("planar_correct_pitch", planar_correct_pitch, 0.0);
    nhPrivate.param<double>("planar_correct_yaw", planar_correct_yaw, 0.0);


    // create the action client
    // true causes the client to spin its own thread
    ac = new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>(gripper_name, true);
    ac->waitForServer(); //will wait for infinite time

    listener = new tf::TransformListener();

    grip_pub = nh.advertise<geometry_msgs::PoseStamped>("grip_pose", 1);
    detectedObjectSub = nh.subscribe("detected_object", 1, detectedObjectCallback);
    arucoDetectedObjectSub = nh.subscribe("markersAruco", 1, arucoDetectedObjectCallback);
    gotoSub = nh.subscribe("goto", 1, gotoCallback);

    if (enablePlanning)
    {
        moveit::planning_interface::MoveGroupInterface::Options options("arm", "robot_description", nh);
        move_group = new moveit::planning_interface::MoveGroupInterface(options);
        geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();


        /*
        ros::Duration(5.0).sleep();
        move_group->setNamedTarget("home");
        move_group->move();
        */
    }

    tf::TransformBroadcaster br;
    tf::StampedTransform correction;
    correction.setIdentity();
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        try
        {
            listener->lookupTransform("world", "checkerboard", ros::Time(0), correction);
            correction.getBasis().getRPY(planar_correct_roll, planar_correct_pitch, planar_correct_yaw);
            correction.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            ROS_INFO_STREAM_THROTTLE(5, "checkerboard correction: " << planar_correct_roll << "," << planar_correct_pitch << "," << planar_correct_yaw);
            correction.setRotation(tf::createQuaternionFromRPY(-planar_correct_pitch, -planar_correct_roll, 0.0));
        }
        catch (tf::TransformException ex)
        {
            //
            correction.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            correction.setRotation(tf::createQuaternionFromRPY(-planar_correct_pitch, -planar_correct_roll, 0.0));
        }
        br.sendTransform(tf::StampedTransform(correction, ros::Time::now(), "winml_link", "winml2_link"));
        loop_rate.sleep();
    }

    ros::waitForShutdown();

    return 0;
}