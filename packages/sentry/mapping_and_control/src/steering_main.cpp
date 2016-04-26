#include <ros/ros.h>
#include <mapping_and_control/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

using namespace std;

geometry_msgs::PoseStamped pose_stamped;
geometry_msgs::Pose pose;
ros::ServiceClient append_client;
ros::ServiceClient flush_client;
ros::ServiceClient estop_client;
ros::ServiceClient clear_estop_client;
ros::ServiceClient lidar_alarm_client;

const int E_STOPPED = 0; //define some mode keywords
const int DONE_W_SUBGOAL = 1;
const int PURSUING_SUBGOAL = 2;
const int HALTING = 3;
int motionMode = -1;

//Used to prevent the robot from getting stuck near a wall by
//setting off the lidar alarm constantly.
//After each time the alarm is sent, you get one subgoal that ignores lidar.
//It should only be used to turn the robot and free it from the wall! 
bool lidar_alarm_sent_recently = false;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
	geometry_msgs::Quaternion quaternion;
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin(phi / 2.0);
	quaternion.w = cos(phi / 2.0);
	return quaternion;
}

void motionModeCallback(const std_msgs::Int32& motionModeMsg) {
	motionMode = motionModeMsg.data;
}

void lidarAlarmCallback(const std_msgs::Bool& lidarAlarmMsg) {

	//does nothing, but is required parameter
	std_srvs::Trigger request;

	switch (motionMode) {
	case E_STOPPED:
	{
		break;
	}
	case HALTING:
	{
		break;
	}
	case PURSUING_SUBGOAL:
	{
		//only set off lidar alarm if the alarm is true,
		//we are on the way to a subgoal,
		//and it hasn't been set off in the last 5 seconds
		//(to allow it to turn away from an obstacle)
		if (lidarAlarmMsg.data && !lidar_alarm_sent_recently) {
			lidar_alarm_client.call(request);
			lidar_alarm_sent_recently = true;
			flush_client.call(request);
		}
		break;
	}
	case DONE_W_SUBGOAL:
	{
		break;
	}
	default: //should not happen
	{
		ROS_WARN("Motion mode: (%d) not recognized!", motionMode);
		break;
	}
	}
}

void pointClickCallback(const geometry_msgs::PointStamped& pointStamped) {

	//does nothing, but is required parameter
	std_srvs::Trigger request;

	bool go = false;

	switch (motionMode) {
	case E_STOPPED:
	{
		ROS_INFO("State: E_STOPPED");
		//clear estop if fully stopped
		clear_estop_client.call(request);
		go = true;
		break;
	}
	case HALTING:
	{
		ROS_INFO("State: HALTING");
		//do nothing, wait for robot to stop
		break;
	}
	case PURSUING_SUBGOAL:
	{
		ROS_INFO("State: PURSUING_SUBGOAL");
		//halt the robot if moving when given a new point
		estop_client.call(request);
		//remove any subgoals in the queue
		flush_client.call(request);
		break;
	}
	case DONE_W_SUBGOAL:
	{
		ROS_INFO("State: DONE_W_SUBGOAL");
		lidar_alarm_sent_recently = false; //reset the lidar alarm, allow it to go off again
		go = true;
		break;
	}
	default: //should not happen
	{
		ROS_WARN("Motion mode: (%d) not recognized!", motionMode);
		break;
	}
	}

	//give it another point to go to
	if (go) {
		mapping_and_control::path path_srv;
		pose.position.x = pointStamped.point.x;
		pose.position.y = pointStamped.point.y;
		pose_stamped.pose = pose;
		path_srv.request.path.poses.push_back(pose_stamped);
		append_client.call(path_srv);
		ROS_INFO("point (%f, %f) appended to path", pointStamped.point.x, pointStamped.point.y);
	}

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "append_path_client");
	ros::NodeHandle n;
	append_client = n.serviceClient<mapping_and_control::path>("append_path_queue_service");
	flush_client = n.serviceClient<std_srvs::Trigger>("flush_path_queue_service");
	estop_client = n.serviceClient<std_srvs::Trigger>("estop_service");
	clear_estop_client = n.serviceClient<std_srvs::Trigger>("clear_estop_service");
	lidar_alarm_client = n.serviceClient<std_srvs::Trigger>("lidar_alarm_service");

	geometry_msgs::Quaternion quat;
	mapping_and_control::path path_srv;

	while (!append_client.exists() || !flush_client.exists() ||
			!estop_client.exists() || !clear_estop_client.exists() ||
			!lidar_alarm_client.exists()) {
		ROS_INFO("waiting for services...");
		ros::Duration(1.0).sleep();
	}
	ROS_INFO("connected client to services");

	pose_stamped.header.frame_id = "world";
	pose.position.x = 0.0;
	pose.position.y = 0.0;
	pose.position.z = 0.0; // let's hope so!
	quat = convertPlanarPhi2Quaternion(0);
	pose.orientation = quat;
	pose_stamped.pose = pose;
	path_srv.request.path.poses.push_back(pose_stamped);


	////TO add a path point and send it, do:
	//	//create some path points... we'll hard-code it for now
	//	pose.position.x = -1.0;
	//	pose.position.y = -1.0;
	//	pose_stamped.pose = pose;
	//	path_srv.request.path.poses.push_back(pose_stamped);
	//
	//	append_client.call(path_srv);


	ros::Subscriber point_click_subscriber = n.subscribe("clicked_point", 1, pointClickCallback);
	ros::Subscriber motion_mode_subscriber = n.subscribe("motion_mode", 1, motionModeCallback);
	ros::Subscriber lidar_subscriber = n.subscribe("lidar_alarm", 1, lidarAlarmCallback);

	ros::spin(); //this is essentially a "while(1)" statement, except it
	// forces refreshing wakeups upon new data arrival
	// main program essentially hangs here, but it must stay alive to keep the callback function alive
	return 0;
}