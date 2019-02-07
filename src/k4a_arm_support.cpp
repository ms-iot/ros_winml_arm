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

using namespace std;
using namespace winrt;

ros::Publisher tracked_object_pub;
ros::Publisher grip_pub;
ros::Publisher goto_pub;
ros::Subscriber markerSub;
ros::Subscriber gotoSub;
tf::TransformListener* listener;

moveit::planning_interface::MoveGroupInterface* move_group;


bool fake = true;
bool enablePlanning = true;

// Engine Block Cylindars
// x = 222mm, 1380mm
// y = 800mm
// z = 360mm, 750mm, 1140mm, 1530mm

double x1 = .0222;
double x2 = .1380;
double xEngineCenter = (x1 + x2) / 2.0;
double yEngineCenter = -.1;
double zEngineCenter = -.15;
double y = .08;

double headLeftOffset = .0945;
double headRightOffset = .087;

void poseForHead(int32_t number, geometry_msgs::Pose& pose)
{
	tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, -M_PI / 2.0);
    pose.position.x = x1;
    pose.position.y = headLeftOffset;
    pose.position.z = y;
	tf::quaternionTFToMsg(q, pose.orientation);
}

// yuck
typedef enum
{
	Mesh,
	XArrow,
	YArrow,
	ZArrow
} MarkerInitType;

void initMarker(visualization_msgs::Marker& marker, visualization_msgs::Marker& relative, std::string name, int32_t id, MarkerInitType type, double x = 0.0, double y = 0.0, double z = 0.0)
{
	marker.header.frame_id = relative.header.frame_id;
    marker.text = name;
	marker.header.stamp = ros::Time();
	marker.ns = "k4a_arm_support";
	marker.id = id;
	marker.type = (type == MarkerInitType::Mesh)?visualization_msgs::Marker::MESH_RESOURCE:visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration();
	marker.mesh_use_embedded_materials = true;
	marker.pose.position.x = x + relative.pose.position.x;
	marker.pose.position.y = y + relative.pose.position.y;
	marker.pose.position.z = z + relative.pose.position.z;
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

void engineMarkerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    auto foundMarker = *msg->markers.begin();

	tf::Quaternion modelPose;
	tf::quaternionMsgToTF(foundMarker.pose.orientation, modelPose);
	tf::Quaternion modelCorrection = tf::createQuaternionFromRPY(0, -M_PI / 2.0, M_PI / 2.0);

	tf::Quaternion gripRotation = modelPose.inverse();

	gripRotation = gripRotation * modelPose * modelCorrection;

	gripRotation.normalize();

    std::vector<visualization_msgs::Marker> markers;
    visualization_msgs::Marker gripPose;
    initMarker(gripPose, foundMarker, "gripPoseX", 0, MarkerInitType::XArrow, xEngineCenter, yEngineCenter, zEngineCenter);
	tf::quaternionTFToMsg(gripRotation, gripPose.pose.orientation);
	markers.push_back(gripPose);
    initMarker(gripPose, foundMarker, "gripPoseY", 1, MarkerInitType::YArrow, xEngineCenter, yEngineCenter, zEngineCenter);
	tf::quaternionTFToMsg(gripRotation, gripPose.pose.orientation);
	markers.push_back(gripPose);
    initMarker(gripPose, foundMarker, "gripPoseZ", 2, MarkerInitType::ZArrow, xEngineCenter, yEngineCenter, zEngineCenter);
	tf::quaternionTFToMsg(gripRotation, gripPose.pose.orientation);
	markers.push_back(gripPose);
	grip_pub.publish(markers);
}

void publishEngine()
{
	std::vector<visualization_msgs::Marker> markers;
    visualization_msgs::Marker marker;

	tf::Quaternion modelCorrection = tf::createQuaternionFromRPY(M_PI, 0, 0);
	modelCorrection.normalize();

	marker.header.frame_id = "winml_link";
	initMarker(marker, marker, "engine", 0, MarkerInitType::Mesh, -0.1, 0, .4);
	tf::quaternionTFToMsg(modelCorrection, marker.pose.orientation);
	markers.push_back(marker);

	tracked_object_pub.publish(markers);
}

void timerCallback(const ros::TimerEvent&)
{
	publishEngine();
}

void gotoCallback(const std_msgs::Int32::ConstPtr& msg)
{
	geometry_msgs::Pose pose;
    poseForHead(msg->data, pose);

	tf::StampedTransform engineToWorld;
	listener->lookupTransform("world", "ee_link", ros::Time(0), engineToWorld);

	tf::Transform engineToTarget;
	tf::poseMsgToTF(pose, engineToTarget);

	auto engineGripPose = (tf::Transform)engineToWorld * engineToTarget;

	//tf::Transform engineGripPose;
	//tf::poseMsgToTF(pose, engineGripPose);

	move_group->setPoseReferenceFrame("world");
	move_group->setGoalTolerance(.0001);

	tf::poseTFToMsg(engineGripPose, pose);

	geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose("ee_link");

	bool planned = move_group->setPoseTarget(pose, "ee_link");
	//bool planned = move_group.setJointValueTarget(pose, "ee_link");
	//bool planned = move_group->setRPYTarget(M_PI / 4.0, 0, 0, "ee_link");
	//bool planned = move_group->setPositionTarget(0.05, 0.0, 0.0, "ee_link");
	//bool planned = move_group->setPositionTarget(current_pose.pose.position.x + .0001, current_pose.pose.position.y, current_pose.pose.position.z, "ee_link");

	/*
	if (!planned)
	{
		// ends up in collision with self, so flip 180. If that doesn't work, bail.
		// rotate the pose

		tf::Quaternion rotateGrip = tf::createQuaternionFromRPY(0, 0, M_PI / 2.0);
		engineGripPose.setRotation(engineGripPose * rotateGrip);

		tf::poseTFToMsg(engineGripPose, pose);
		planned = move_group.setPoseTarget(pose);
		//planned = move_group.setJointValueTarget(pose, "ee_link");
	}
	*/

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	planned = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", planned ? "" : "FAILED");

	if (planned)
	{
		move_group->move();
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


    bool f; 
    if (nhPrivate.getParam("fake", f))
    {
        fake = f;
    }

	bool p;
	if (nhPrivate.getParam("enablePlanning", p))
	{
		enablePlanning = p;
	}
	

	listener = new tf::TransformListener();

	grip_pub = nh.advertise<visualization_msgs::MarkerArray>("grip_pose", 1);
	goto_pub = nh.advertise<std_msgs::Int32>("goto", 1);

	markerSub = nh.subscribe("tracked_objects", 1, engineMarkerCallback);

    gotoSub = nh.subscribe("goto", 1, gotoCallback);

	if (enablePlanning)
	{
		moveit::planning_interface::MoveGroupInterface::Options options("arm", "robot_description", nh);
		move_group = new moveit::planning_interface::MoveGroupInterface(options);
		geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();

		ros::Duration(5.0).sleep();

		move_group->setNamedTarget("home");
		move_group->move();
	}
	/*
	if (fake)
	{
		timer = nh.createTimer(ros::Duration(1), timerCallback);
		tracked_object_pub = nh.advertise<visualization_msgs::MarkerArray>("tracked_objects", 1);

		std_msgs::Int32 dummy;
		goto_pub.publish(dummy);
	}
	*/

    ros::waitForShutdown();

    return 0;
}