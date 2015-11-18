#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>

using namespace ros;
using namespace std_msgs;
using namespace visualization_msgs;

void pressure_callback(const Float32::ConstPtr& pressure, Publisher* pub_marker)
{
	// Create marker message and fill some info
	Marker marker;
	marker.header.frame_id = "/end_effector";
	marker.type = marker.SPHERE;
	marker.action = marker.ADD;
	marker.color.a = 0.6;

	// Factor for scale and color
	float factor = std::min(pressure->data / 0.5, 1.0);

	// Scale
	float scale_factor = 0.12 + 0.1 * factor;
	marker.scale.x = scale_factor;
	marker.scale.y = scale_factor;
	marker.scale.z = scale_factor;

	// Color
	marker.color.g = (1-factor);
	marker.color.r = factor;

	// Publish marker
	pub_marker->publish(marker);
}

int main(int argc, char** argv)
{
	// Init node
	ros::init(argc, argv, "robostylus_pressure_marker");
	NodeHandle nh("~");

	// Create marker publisher and pressure info subscriber
	Publisher pub_marker = nh.advertise<Marker>("pressure_marker", 10);
	Subscriber sub_pressure = nh.subscribe<Float32>("/robostylus_driver/pressure", 10, boost::bind(pressure_callback, _1, &pub_marker));

	// Spin callbacks
	ros::spin();

	return 0;
}
