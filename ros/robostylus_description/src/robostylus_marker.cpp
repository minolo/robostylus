#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <actionlib/client/simple_action_client.h>

#include <robostylus_description/RobostylusMoveAction.h>

using namespace ros;
using namespace visualization_msgs;
using namespace interactive_markers;
using namespace geometry_msgs;

using namespace robostylus_description;

typedef actionlib::SimpleActionClient<RobostylusMoveAction> RobostylusActionClient;

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	if(feedback->event_type == InteractiveMarkerFeedback::BUTTON_CLICK)
	{
		ROS_INFO("Marker clicked at (%f, %f, %f)", feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);

		RobostylusMoveGoal goal;

		// Create goal for the driver action server
		goal.position.x = feedback->pose.position.x;
		goal.position.y = feedback->pose.position.y;
		goal.position.z = feedback->pose.position.z;
		goal.pressure_limit.data = 0.5;
		goal.ticks.data = 40;

		ros::Duration timeout(1.0);

		// Send the goal
		RobostylusActionClient client_action("robostylus_driver/robostylus_move");
		client_action.waitForServer();

		client_action.sendGoal(goal);
		client_action.waitForResult(timeout);

		if(client_action.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_WARN("The goal was not successful");
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robostylus_marker");

	NodeHandle nh("~");

	InteractiveMarkerServer server("robostylus_marker");

	// Create marker object
	InteractiveMarker int_marker;
	int_marker.header.frame_id = "/base";
	int_marker.name = "position_marker";
	int_marker.description = "";
	int_marker.scale = 0.3;
	int_marker.pose.position.x = 1;
	int_marker.pose.position.y = 0;
	int_marker.pose.position.z = 0.3;

	// Create marker geometry
	Marker marker;
	marker.type = Marker::SPHERE;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.5;
	marker.color.a = 1.0;

	// Visibility control
	InteractiveMarkerControl marker_control;
	marker_control.always_visible = true;

	int_marker.controls.push_back(marker_control);

	// Create control to move in X axis
	InteractiveMarkerControl move_control;
	move_control.orientation.w = 1;
	move_control.orientation.x = 1;
	move_control.orientation.y = 0;
	move_control.orientation.z = 0;
	move_control.name = "move_x";
	move_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(move_control);

	// Create control to move in Y axis
	move_control.orientation.w = 1;
	move_control.orientation.x = 0;
	move_control.orientation.y = 1;
	move_control.orientation.z = 0;
	move_control.name = "move_y";
	move_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(move_control);

	// Create control to move in Z axis
	move_control.orientation.w = 1;
	move_control.orientation.x = 0;
	move_control.orientation.y = 0;
	move_control.orientation.z = 1;
	move_control.name = "move_z";
	move_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(move_control);

	// Create button control
	InteractiveMarkerControl button_control;
	button_control.interaction_mode = InteractiveMarkerControl::BUTTON;
	button_control.name = "button_control";
	button_control.markers.push_back(marker);
	int_marker.controls.push_back(button_control);

	// Add marker
	server.insert(int_marker, (InteractiveMarkerServer::FeedbackCallback)&processFeedback);

	server.applyChanges();

	ros::spin();

	return 0;
}
