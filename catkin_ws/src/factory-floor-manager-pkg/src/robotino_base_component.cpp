/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include <functional>
#include <memory>
#include <thread>
#include <algorithm>
#include <mutex>

#include <actionlib/client/simple_action_client.h>

#include "robotino_local_move/LocalMoveAction.h"

#include "two_d_move_capability.h"
#include "component.h"
#include "robotino_base_component.h"

std::mutex g_state_mutex;

RobotinoBaseComponent::RobotinoBaseComponent(const ros::NodeHandle& parentNodeHandle)
				: Component(), _nh(parentNodeHandle, "robotino_base"), _previous_feedback(0.0, 0.0, 0.0) {
	_state_ptr = std::make_shared<State>(IDLE);
	_publisher = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	_odometry_subscriber  = _nh.subscribe( "/odom", 1, &RobotinoBaseComponent::odometryCallback, this );
	TwoDMoveCapabilityCallback two_d_move_capability_callback = [this](std::shared_ptr<TwoDMoveCapabilityParameters> parameters) -> void {
		RobotinoBaseComponent::act(parameters);
	};
	std::shared_ptr<TwoDMoveCapability> two_d_move_capability_ptr = std::make_shared<TwoDMoveCapability>(two_d_move_capability_callback);
	_capabilities_vector.push_back(two_d_move_capability_ptr);
}

std::pair<bool, std::shared_ptr<Capability>> RobotinoBaseComponent::findCapabilityByName(
				const std::string& capability_name) {
	for (std::shared_ptr<Capability> capability_ptr : _capabilities_vector) {
		if (capability_ptr->isNamed(capability_name)) {
			return std::make_pair<bool, std::shared_ptr<Capability>>(true, std::move(capability_ptr));
		}
	}
	// When no capability was found, we return false and an empty shared ptr.
	return std::make_pair<bool, std::shared_ptr<Capability>>(false, std::shared_ptr<Capability>());
}

void RobotinoBaseComponent::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
	_curr_x = msg->pose.pose.position.x;
	_curr_y = msg->pose.pose.position.y;
}

void RobotinoBaseComponent::localMoveFeedbackCallback(
				const robotino_local_move::LocalMoveFeedbackConstPtr& feedback_ptr) {
	auto new_feedback = std::make_tuple(
					feedback_ptr->forward_dist_x, feedback_ptr->forward_dist_y, feedback_ptr->rotation_dist);
	if (new_feedback == _previous_feedback) {
		ROS_INFO("%s, %d: The robotino doesn't seem to be moving...", __FILE__, __LINE__);
	}
}

bool RobotinoBaseComponent::isBusy() {
	return *_state_ptr == BUSY;
}

void RobotinoBaseComponent::act(std::shared_ptr<TwoDMoveCapabilityParameters> parameters) {
	ROS_INFO("%s, %d: Executing TwoDMove", __FILE__, __LINE__);
	changeState(BUSY);
	std::vector<std::string> nodes;
	ros::master::getNodes(nodes);
	if (std::find(nodes.begin(), nodes.end(), "/robotino_node") == nodes.end()) {
		ROS_ERROR("%s, %d: /robotino_node is not online!", __FILE__, __LINE__);
		return;
	}
	actionlib::SimpleActionClient<robotino_local_move::LocalMoveAction> client("local_move");
	// BUSY HERE
	if (!client.waitForServer(ros::Duration(2.0))) {
		ROS_ERROR("%s, %d: Could not connect to robotino local move action server!", __FILE__, __LINE__);
	}
	if (*_state_ptr == RobotinoBaseComponent::State::BUSY) {
		ROS_INFO("%s, %d: BUSY", __FILE__, __LINE__);
	}
	else {
		ROS_INFO("%s, %d: IDLE", __FILE__, __LINE__);
	}
	// IDLE HERE
	robotino_local_move::LocalMoveGoal client_goal;
	client_goal.move_x = parameters->to_x;
	client_goal.move_y = parameters->to_y;
	client_goal.rotation = parameters->to_rotation;
	client.sendGoal(client_goal, actionlib::SimpleActionClient<robotino_local_move::LocalMoveAction>::SimpleDoneCallback()
					, actionlib::SimpleActionClient<robotino_local_move::LocalMoveAction>::SimpleActiveCallback()
					, boost::bind(&RobotinoBaseComponent::localMoveFeedbackCallback, this, _1));
	while(!client.waitForResult(ros::Duration(3.0))) {
		ROS_INFO("Waiting for the robotino to finish its task...");
	}
	changeState(IDLE);
}

void RobotinoBaseComponent::changeState(RobotinoBaseComponent::State new_state) {
	g_state_mutex.lock();
	*_state_ptr = new_state;
	g_state_mutex.unlock();
}

RobotinoBaseComponent::RobotinoBaseComponent(const RobotinoBaseComponent& other) {
	_state_ptr = other._state_ptr;
	_nh = other._nh;
	_publisher = other._publisher;
	_odometry_subscriber = other._odometry_subscriber;
	_curr_x = _start_x = _dist_moved = _curr_y = _start_y = 0.0;
	_previous_feedback = other._previous_feedback;
	_capabilities_vector = other._capabilities_vector;
}

RobotinoBaseComponent& RobotinoBaseComponent::operator=(const RobotinoBaseComponent& other) {
	if (this != &other) {
		_state_ptr = other._state_ptr;
		_nh = other._nh;
		_publisher = other._publisher;
		_odometry_subscriber = other._odometry_subscriber;
		_curr_x = _start_x = _dist_moved = _curr_y = _start_y = 0.0;
		_previous_feedback = other._previous_feedback;
	}
	return *this;
}

RobotinoBaseComponent::~RobotinoBaseComponent() {
	
}

const std::vector<std::shared_ptr<Capability>>& RobotinoBaseComponent::getCapabilities() {
	return _capabilities_vector;
}

std::string RobotinoBaseComponent::getName() {
	return "RobotinoBaseComponent";
}

