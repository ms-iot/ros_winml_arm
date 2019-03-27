#define _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS 1 // The C++ Standard doesn't provide equivalent non-deprecated functionality yet.
#pragma comment(lib, "windowsapp")

#include <ros/ros.h>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>
#include <vcruntime.h>
#include <windows.h>
#include <winrt/Windows.Foundation.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Int32.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>
#include <winml_msgs/DetectedObjectPose.h>

using namespace std;
using namespace winrt;

ros::Publisher tracked_object_pub;
ros::Publisher grip_pub;
ros::Subscriber gotoSub;
ros::Subscriber detectedObjectSub;
tf::TransformListener* listener;

moveit::planning_interface::MoveGroupInterface* move_group;
actionlib::SimpleActionClient<control_msgs::GripperCommandAction>* ac;


bool fake = true;
bool enablePlanning = true;

// Engine Block Cylindars
// x = 222mm, 1380mm
// y = 800mm
// z = 360mm, 750mm, 1140mm, 1530mm

double x1 = .0222;
double x2 = .1380;
double xEngineCenter = 0;
double yEngineCenter = 0;
double zEngineCenterFirst = .3;
double zEngineCenterSecond = .28;
double y = .08;

double correctionR = M_PI;
double correctionP = 0.0;
double correctionY = M_PI / 2.0;


ros::WallTime lastSeen;
tf::Transform currentGripPose;

// yuck
typedef enum
{
    Mesh,
    XArrow,
    YArrow,
    ZArrow
} MarkerInitType;

void initMarker(visualization_msgs::Marker& marker, std::string name, int32_t id, MarkerInitType type, double x = 0.0, double y = 0.0, double z = 0.0)
{
    marker.header.frame_id = "world";
    marker.text = name;
    marker.header.stamp = ros::Time();
    marker.ns = "k4a_arm_support";
    marker.id = id;
    marker.type = (type == MarkerInitType::Mesh)?visualization_msgs::Marker::MESH_RESOURCE:visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    marker.mesh_use_embedded_materials = true;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.points.clear();
    
    marker.color.a = 1.0;
    marker.color.r = 0.0; 	marker.color.g = 0.0;	marker.color.b = 0.0;
    if (type != MarkerInitType::Mesh)
    {
        geometry_msgs::Point pt;
        marker.points.push_back(pt);
        switch (type)
        {
            case MarkerInitType::ZArrow:
                marker.color.b = 1.0;
                pt.z = .1;
                break;

            case MarkerInitType::YArrow:
                marker.color.g = 1.0;
                pt.y = .1;
                break;

            case MarkerInitType::XArrow:
                marker.color.r = 1.0;
                pt.x = .1;
                break;
        }
        marker.points.push_back(pt);

        marker.scale.x = .01;
        marker.scale.y = .012;
        marker.scale.z = 0.0;
    }
    else
    {
        marker.mesh_resource = "package://k4a_arm_support/meshes/Engine_Block.stl";
        marker.scale.x = .001;
        marker.scale.y = .001;
        marker.scale.z = .001;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }

}

void openGripper()
{
  control_msgs::GripperCommandGoal goal;
  goal.command.position = 0.08;
  goal.command.max_effort = 30.0;
  ac->sendGoal(goal);
}

void closeGripper()
{
  control_msgs::GripperCommandGoal goal;
  goal.command.position = -0.017;
  goal.command.max_effort = 30.0;
  ac->sendGoal(goal);
  ac->waitForResult(ros::Duration(30.0));
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

    auto engineGripPose = (tf::Transform)engineToWorld * enginePose;

    engineGripPose.getOrigin() += gripOffset;

    tf::Quaternion modelCorrection = tf::createQuaternionFromRPY(correctionR, correctionP, correctionY);

    auto gripRotationQ = engineGripPose * modelCorrection;
    gripRotationQ.normalize();

    std::vector<visualization_msgs::Marker> markers;
    visualization_msgs::Marker gripPose;
    initMarker(gripPose, "gripPoseX", 0, MarkerInitType::XArrow, engineGripPose.getOrigin().x(), engineGripPose.getOrigin().y(), engineGripPose.getOrigin().z());
    tf::quaternionTFToMsg(gripRotationQ, gripPose.pose.orientation);
    markers.push_back(gripPose);
    initMarker(gripPose, "gripPoseY", 1, MarkerInitType::YArrow, engineGripPose.getOrigin().x(), engineGripPose.getOrigin().y(), engineGripPose.getOrigin().z());
    tf::quaternionTFToMsg(gripRotationQ, gripPose.pose.orientation);
    markers.push_back(gripPose);
    initMarker(gripPose, "gripPoseZ", 2, MarkerInitType::ZArrow, engineGripPose.getOrigin().x(), engineGripPose.getOrigin().y(), engineGripPose.getOrigin().z());
    tf::quaternionTFToMsg(gripRotationQ, gripPose.pose.orientation);
    markers.push_back(gripPose);
    grip_pub.publish(markers);

    currentGripPose = engineGripPose;
    currentGripPose.setRotation(gripRotationQ);

    lastSeen = ros::WallTime::now();
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
        // Pickup Mode
        if (ros::WallTime::now() > (lastSeen + ros::WallDuration(30, 0)))
        {
            // Didn't see soon enough
            return;
        }

        move_group->setPoseReferenceFrame("world");
        move_group->setGoalTolerance(.005);

        geometry_msgs::Pose gripPoseMsg;
        tf::poseTFToMsg(currentGripPose, gripPoseMsg);

        move_group->setPoseTarget(gripPoseMsg, "ee_link");

        moveit::planning_interface::MoveGroupInterface::Plan gripPlan;

        bool planned = (move_group->plan(gripPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("k4a", "plan 1 (pose goal) %s", planned ? "" : "FAILED");

        if (!planned)
        {
            // ends up in collision with self, so flip 180. If that doesn't work, bail.
            // rotate the pose

            tf::Quaternion rotateGrip = tf::createQuaternionFromRPY(0, 0, M_PI / 2.0);

            currentGripPose.setRotation(currentGripPose.getRotation() * rotateGrip);

            tf::poseTFToMsg(currentGripPose, gripPoseMsg);
            move_group->setPoseTarget(gripPoseMsg, "ee_link");
            planned = (move_group->plan(gripPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            ROS_INFO_NAMED("k4a", "plan 2 (pose goal) %s", planned ? "" : "FAILED");
        }

        if (planned)
        {
            move_group->move();
        }

        gripPoseMsg.position.z = zEngineCenterSecond;

        planned = move_group->setPoseTarget(gripPoseMsg, "ee_link");
        planned = (move_group->plan(gripPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("k4a", "grip (pose goal) %s", planned ? "" : "FAILED");

        if (planned)
        {
            move_group->move();
            closeGripper();

            move_group->setNamedTarget("home");
            move_group->move();
        }
    }
    else if (msg->data == 2)
    {
        move_group->setNamedTarget("place");
        move_group->move();
        openGripper();
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
    ros::Timer timer;
    ros::AsyncSpinner async_spinner (2);
    async_spinner.start();

    lastSeen = ros::WallTime::now() - ros::WallDuration(60);	// last seen while ago

    std::string gripper_name;
      nhPrivate.param<std::string>("gripper_name", gripper_name, "gripper");
    nhPrivate.param<bool>("enablePlanning", enablePlanning, true);


    // create the action client
      // true causes the client to spin its own thread
      ac = new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>(gripper_name, true);
    ac->waitForServer(); //will wait for infinite time

    listener = new tf::TransformListener();

    grip_pub = nh.advertise<visualization_msgs::MarkerArray>("grip_pose", 1);
    detectedObjectSub = nh.subscribe("detected_object", 1, detectedObjectCallback);

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

    ros::waitForShutdown();

    return 0;
}