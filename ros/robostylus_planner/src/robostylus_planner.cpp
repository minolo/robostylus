#include <queue>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <robostylus_description/RobostylusMoveAction.h>
#include <robostylus_planner/Point2D.h>
#include <actionlib/client/simple_action_client.h>

#include <robostylus_camera/TransformPoint.h>

using namespace std;
using namespace ros;
using namespace robostylus_description;
using namespace robostylus_planner;

#define CONSOLE_HEIGHT 0.14

// States for the state machine
typedef enum
{
	STATE_IDLE, STATE_POINT_GO, STATE_POINT_TOUCH, STATE_POINT_DONE, STATE_RETURN, STATE_PARK
} planner_state_t;

// Current planner state
planner_state_t planner_state;

// Client for robot actions
typedef actionlib::SimpleActionClient<RobostylusMoveAction> RobostylusActionClient;

// Declare pre-parking and parking positions for the robot arm
tf::Vector3 position_parking(0.3, 0, 0.6);
tf::Vector3 position_pre_parking(0.6, 0, CONSOLE_HEIGHT + 0.2);

queue<tf::Vector3> points;
void callback_point(const Point2D::ConstPtr& point2d_msg, ServiceClient* client_transformPoint, tf::TransformListener* tf_listener)
{
	// Project the point
	robostylus_camera::TransformPoint tp;
	tp.request.x = point2d_msg->x;
	tp.request.y = point2d_msg->y;

    // Wait for transform service
	client_transformPoint->waitForExistence();

    // Call service
	if(client_transformPoint->call(tp) && tp.response.success)
	{
		try
		{
			// Get frame names
			string base_frame = "/base";
			string camera_frame = tp.response.camera_frame;

			// Obtain transform from camera frame to base frame
			tf::StampedTransform t_camera_to_base;
			tf_listener->waitForTransform(base_frame, camera_frame, ros::Time::now(), ros::Duration(1));
			tf_listener->lookupTransform(base_frame, camera_frame, ros::Time(0), t_camera_to_base);

			// Get camera ray projection of the input point
			tf::Vector3 camera_ray;
			camera_ray.setX(tp.response.x);
			camera_ray.setY(tp.response.y);
			camera_ray.setZ(tp.response.z);

			// Get camera ray projection in the base frame
			tf::Vector3 camera_position = t_camera_to_base.getOrigin();
			tf::Vector3 base_ray = t_camera_to_base(camera_ray) - camera_position;
			ROS_DEBUG("Camera ray: (%lf, %lf, %lf)", base_ray.getX(), base_ray.getY(), base_ray.getZ());

			// Obtain the point position in 3D space
			float camera_height = camera_position.getZ() - CONSOLE_HEIGHT;
			float factor = camera_height / -base_ray.getZ();
			tf::Vector3 position = camera_position + base_ray * factor;

			ROS_DEBUG("Point position: (%lf, %lf, %lf)", position.getX(), position.getY(), position.getZ());

			// Store the point
			points.push(position);
		}
		catch(tf::TransformException& ex)
		{
			ROS_WARN("robostylus_planner TF exception: %s", ex.what());
		}
	}
	else
	{
		ROS_WARN("Failure when projecting point (%d, %d)", point2d_msg->x, point2d_msg->y);
	}
}

int go_to_position(RobostylusActionClient& client_action, tf::Vector3 position, float pressure_limit, int ticks)
{
	RobostylusMoveGoal goal;

	// Fill request data
	goal.position.x = position.x();
	goal.position.y = position.y();
	goal.position.z = position.z();
	goal.pressure_limit.data = pressure_limit;
	goal.ticks.data = ticks;

	// Calculate max timeout
	ros::Duration timeout(((float)std::abs(ticks) / 76) + 1.0);

	// Send the goal to the driver node
	client_action.sendGoal(goal);

	// Wait for an answer from the driver node
	while(ros::ok())
	{
		if(!client_action.waitForResult(timeout))
		{
			ROS_WARN_THROTTLE(10, "Timeout when waiting for goal");
		}
		else if(client_action.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_WARN_THROTTLE(10, "A goal is being preempted");
		}
		else
		{
			break;
		}

		sleep(1);

		client_action.sendGoal(goal);
	}

	return client_action.getResult()->ticks_taken.data;
}

int main(int argc, char** argv)
{
	// Initialize node
	ros::init(argc, argv, "robostylus_planner");
	NodeHandle nh("~");

	// Create action server
	RobostylusActionClient client_action("robostylus_driver/robostylus_move");
	client_action.waitForServer();

	// Create point transform client
	ServiceClient client_transformPoint = nh.serviceClient<robostylus_camera::TransformPoint>("/robostylus_camera/transform_point");

	// Create TF listener
	tf::TransformListener tf_listener;

	// Subscribe to screen point topic
	Subscriber sub_points = nh.subscribe<Point2D>("points", 10, boost::bind(callback_point, _1, &client_transformPoint, &tf_listener));

	planner_state = STATE_PARK;

	int last_ticks_taken = 0;
	ros::Duration duration_sleep(0.5);
	while(ros::ok())
	{
		switch(planner_state)
		{
			case STATE_IDLE:
				if(!points.empty())
				{
					// Go to adjusted pre-park position
					//tf::Vector3 pre_parking_adjusted = position_pre_parking + tf::Vector3(0, points.front().y(), 0);
					//go_to_position(client_action, pre_parking_adjusted, 1, 30);
					go_to_position(client_action, position_pre_parking, 1, 30);
					planner_state = STATE_POINT_GO;
				}
				else
				{
					duration_sleep.sleep();
				}
				break;
			case STATE_POINT_GO:
				// Go to point above target
				go_to_position(client_action, points.front() + tf::Vector3(0,0,0.2), 1, -70);
				planner_state = STATE_POINT_TOUCH;
				break;
			case STATE_POINT_TOUCH:
				// Go to point
				last_ticks_taken = go_to_position(client_action, points.front() + tf::Vector3(0,0,-0.05), 0.6, 50);
				last_ticks_taken = max(last_ticks_taken, 10);
				planner_state = STATE_POINT_DONE;
				break;
			case STATE_POINT_DONE:
				// Go to point above target
				go_to_position(client_action, points.front() + tf::Vector3(0,0,0.2), 1, last_ticks_taken);

				points.pop();
				if(!points.empty())
				{
					planner_state = STATE_POINT_GO;
				}
				else
				{
					planner_state = STATE_RETURN;
				}
				break;
			case STATE_RETURN:
				// Go to pre-park position
				go_to_position(client_action, position_pre_parking, 1, -80);
				if(!points.empty())
				{
					planner_state = STATE_POINT_GO;
				}
				else
				{
					planner_state = STATE_PARK;
				}
				break;
			case STATE_PARK:
				// Go to park position
				go_to_position(client_action, position_parking, 1, 60);
				planner_state = STATE_IDLE;
				break;
		}

		ros::spinOnce();
	}

	return 0;
}
