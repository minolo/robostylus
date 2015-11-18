#include <boost/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <actionlib/server/simple_action_server.h>
#include <Eigen/Core>

#include <robostylus_description/RobostylusMoveAction.h>

#include "serial/RSSerialController.h"

using namespace std;
using namespace ros;
using namespace KDL;
using namespace std_msgs;
using namespace sensor_msgs;
using namespace robostylus_description;
using namespace Eigen;

typedef actionlib::SimpleActionServer<RobostylusMoveAction> RobostylusActionServer;

// Action server for robot movement
RobostylusActionServer* server;

// Sensor message publishers
Publisher pub_joints;
Publisher pub_pressure;

// Joint names and current joint states
vector<string> joint_names;
JntArray q_current(NUM_SERVOS);

// KDL tree, chain and solvers
Tree tree;
Chain chain;
Eigen::Matrix<double,6,1> L;
ChainIkSolverPos_LMA *ikSolverPos;
ChainFkSolverPos_recursive *fkSolverPos;

// Eigen matrices for least squares approximations
Matrix<float, Dynamic, 3> 	X;
Matrix<float, Dynamic, 1> 	Y[NUM_SERVOS];
Matrix<float, 3, 1> 		A[NUM_SERVOS];

void goalCallback()
{
	int ret;

	// Accept the new goal
	boost::shared_ptr<const RobostylusMoveGoal> goal = server->acceptNewGoal();

	// Calculate current end effector position
	Frame current_frame;
	ret = fkSolverPos->JntToCart(q_current, current_frame);
	if(ret < 0)
	{
		ROS_WARN("FkSolverPos returned %d", ret);
		server->setPreempted();
		return;
	}

	ROS_DEBUG_NAMED("driver", "POS: %f %f %f", current_frame.p.x(), current_frame.p.y(), current_frame.p.z());
	ROS_DEBUG_NAMED("driver", "GOTO: %f %f %f", goal->position.x, goal->position.y, goal->position.z);

	// Calculate difference between final and initial positions
	Vector difference(goal->position.x - current_frame.p.x(), goal->position.y - current_frame.p.y(), goal->position.z - current_frame.p.z());

	// Get the number of intermediate positions to calculate IK for
	int ticks;
	if(goal->ticks.data > 0)
	{
		// If ticks is positive, it represents the total amount of ticks the movement should take
		ticks = goal->ticks.data;
	}
	else
	{
		// If ticks is negative, it means ticks per unit of distance
		ticks = (int)(-goal->ticks.data * difference.Norm());
	}

	// 3 ticks is the minimum because a polynomial of degree 2 is used
	ticks = std::max<int>(3, ticks);

	// Prepare matrices
	X.resize(ticks, Eigen::NoChange);
	for(int i = 0; i < NUM_SERVOS; i++)
	{
		Y[i].resize(ticks, Eigen::NoChange);
	}

	// Fill matrices with angle data
	Frame destination(Rotation::RPY(0, 0, 0), Vector());
	for(int i = 0; i < ticks; i++)
	{
		// Calculate frame for current tick
		destination.p = current_frame.p + ((difference * i) / (ticks - 1));
		destination.M = Rotation::RotY(current_frame.M.GetRot().y() * (1 - (((float)i) / (ticks - 1))));

		// Calculate joints for current position
		JntArray q_init(NUM_SERVOS), q_out(NUM_SERVOS);
		ret = ikSolverPos->CartToJnt(q_init, destination, q_out);
		if(ret < 0)
		{
			ROS_WARN("IkSolverPos returned %d at tick %d", ret, i);
			server->setPreempted();
			return;
		}

		// Fill current position data
		X(i, 0) = 1;
		X(i, 1) = i;
		X(i, 2) = i * i;

		for(int servo = 0; servo < NUM_SERVOS; servo++)
		{
			Y[servo](i, 0) = q_out(servo);
		}
	}

	// Calculate best approximation
	MatrixXf Xt = X.transpose();
	MatrixXf XtXi_Xt = (Xt * X).inverse() * Xt;

	for(int servo = 0; servo < NUM_SERVOS; servo++)
	{
		A[servo] = XtXi_Xt * Y[servo];
	}

	// Check accuracy of approximations
	float error = 0;
	for(int i = 0; i < ticks; i++)
	{
		for(int servo = 0; servo < NUM_SERVOS; servo++)
		{
			error += abs((float)(Y[servo](i, 0) - (A[servo](2, 0) * i * i + A[servo](1, 0) * i + A[servo](0, 0))));
		}
	}
	ROS_DEBUG_NAMED("driver", "ERROR: %f", error);

	// Check if the device is connected
	if(!RSSerialController::connected())
	{
		ROS_WARN_THROTTLE(10, "The device is not connected!");
		server->setPreempted();
	}
	else
	{
		// Configure servo coefficients
		for(int servo = 0; servo < NUM_SERVOS; servo++)
		{
			RSSerialController::setCoeffs(servo, A[servo].coeff(2, 0), A[servo].coeff(1, 0), A[servo].coeff(0, 0));
		}

		// Configure pressure limit and tick count, and start the animation
		RSSerialController::setPressureLimit(goal->pressure_limit.data);
		RSSerialController::setTickCount(ticks);
		RSSerialController::start();
	}
}

void sensorPacketCallback(vector<float>& joints_pos, vector<float>& joints_adc, float pressure)
{
	// Send joint state message
	JointState msg_joints;

	msg_joints.header.stamp = Time::now();

	for(int i = 0; i < joints_adc.size(); i++)
	{
		msg_joints.name.push_back(joint_names[i]);
		msg_joints.position.push_back(joints_adc[i]);
	}

	pub_joints.publish(msg_joints);

	// Send pressure message
	Float32 msg_pressure;
	msg_pressure.data = pressure;
	pub_pressure.publish(msg_pressure);

	// Store current positions for IK computations
	for(int i = 0; i < joints_pos.size(); i++)
	{
		q_current(i) = joints_pos[i];
	}
}

void movementFinishedPacketCallback(unsigned short ticks_taken)
{
	if(server->isActive())
	{
		RobostylusMoveResult result;
		result.ticks_taken.data = ticks_taken;
		server->setSucceeded(result);
	}
}

void getParameters(NodeHandle &nh)
{
	////////////////////////////////////////////////////////////
	// Get robot description
	////////////////////////////////////////////////////////////
	string description;
	if(!nh.getParam("/robot_description", description))
	{
		ROS_ERROR("Parameter not set: /robot_description");
		exit(-1);
	}

	////////////////////////////////////////////////////////////
	// Create robot model
	////////////////////////////////////////////////////////////
	urdf::Model model;
	if(!model.initString(description))
	{
		ROS_ERROR("Could not initialize robot model");
		exit(-1);
	}

	////////////////////////////////////////////////////////////
	// Get tip link name and verify it exists in the model
	////////////////////////////////////////////////////////////
	string tip_link_name;
	if(!nh.getParam("tip_link_name", tip_link_name))
	{
		ROS_ERROR("Parameter not set: %s/tip_link_name", nh.getNamespace().c_str());
		exit(-1);
	}
	if(model.links_.find(tip_link_name) == model.links_.end())
	{
		ROS_ERROR("The link specified in %s/tip_link_name does not exist", nh.getNamespace().c_str());
		exit(-1);
	}

	////////////////////////////////////////////////////////////
	// Get revolute joints
	////////////////////////////////////////////////////////////
	vector<boost::shared_ptr<urdf::Joint> > model_joints;

	for(boost::shared_ptr<const urdf::Link> current_link = model.getLink(tip_link_name); current_link->parent_joint != NULL; current_link = current_link->getParent())
	{
		if(current_link->parent_joint->type == urdf::Joint::REVOLUTE)
		{
			model_joints.insert(model_joints.begin(), current_link->parent_joint);
		}
	}

	if(model_joints.size() != NUM_SERVOS)
	{
		ROS_ERROR("The robot model must have %d revolute joints, found %d", NUM_SERVOS, (int)model_joints.size());
		exit(-1);
	}

	////////////////////////////////////////////////////////////
	// Configure servo limits
	////////////////////////////////////////////////////////////
	for(int joint_i = 0; joint_i < NUM_SERVOS; joint_i++)
	{
		float lowerLimit = model_joints[joint_i]->limits->lower;
		float upperLimit = model_joints[joint_i]->limits->upper;

		RSSerialController::setServoLimits(joint_i, lowerLimit, upperLimit);
	}

	////////////////////////////////////////////////////////////
	// Store joint names
	////////////////////////////////////////////////////////////
	for(int joint_i = 0; joint_i < NUM_SERVOS; joint_i++)
	{
		joint_names.push_back(model_joints[joint_i]->name);
	}

	////////////////////////////////////////////////////////////
	// Create KDL tree
	////////////////////////////////////////////////////////////
	if(!kdl_parser::treeFromString(description, tree))
	{
		ROS_ERROR("Failed to construct kdl tree");
		exit(-1);
	}

	if(!tree.getChain(model.getRoot()->name, tip_link_name, chain))
	{
		ROS_ERROR("Failed to make chain from tree");
		exit(-1);
	}
}

int main(int argc, char** argv)
{
	// Init ROS node
	ros::init(argc, argv, "robostylus_driver");

	// Get node handle
	NodeHandle nh("~");

	// Initialize serial controller and callbacks
	RSSerialController::init(nh);
	RSSerialController::setSensorPacketCallback(sensorPacketCallback);
	RSSerialController::setMovementFinishedPacketCallback(movementFinishedPacketCallback);

	// Get ROS parameters
	getParameters(nh);

	// Create KDL solvers
	L(0) = 1; L(1) = 1; L(2) = 1; L(3) = 1; L(4) = 1; L(5) = 0;
	ikSolverPos = new KDL::ChainIkSolverPos_LMA(chain, L);
	fkSolverPos = new KDL::ChainFkSolverPos_recursive(chain);

	// Create sensor feedback publishers
	pub_joints = nh.advertise<JointState>("joints", 10);
	pub_pressure = nh.advertise<Float32>("pressure", 10);

	// Create action server
	server = new RobostylusActionServer(nh, "robostylus_move", false);
	server->registerGoalCallback((boost::function<void()>)goalCallback);
	server->start();

	// Start serial read thread
	boost::thread t = boost::thread(RSSerialController::run);

	// Spin callbacks
	ros::spin();

	return 0;
}
