/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include <thread>

#include <actionlib/client/simple_action_client.h>

#include "agent_controller.h"
#include "robotino_controller.h"
#include "factory_floor_manager/status_reportResult.h"
#include "robotino_two_d_move_step.h"
#include "step_server.h"
#include "robotino_local_move/LocalMoveAction.h"

StepServer::StepServer(ros::NodeHandle& parent_nh, std::vector<std::shared_ptr<AgentController>>& controller_ptrs)
		: _nh{parent_nh, "step_server"}
		, _robotino_two_d_move_step_as{
								_nh, "robotino_two_d_move_step_server", false
				}
		, _status_report_server(
				_nh, "status_report_server", false
		)
		, _controllers_ptr(&controller_ptrs)
		, _waiting_steps()
		, _currently_executed_steps()
{
	_status_report_server.registerGoalCallback(boost::bind(&StepServer::statusCallback, this));
	_status_report_server.start();
}

void StepServer::robotinoTwoDMoveGoalCallback() {
	// Receive and handle the step
	factory_floor_manager::robotino_two_d_move_stepGoalConstPtr goal_ptr;
	if (!_robotino_two_d_move_step_as.isPreemptRequested()) {
		goal_ptr = _robotino_two_d_move_step_as.acceptNewGoal();
	}
	
	factory_floor_manager::robotino_two_d_move_stepFeedback feedback;
	feedback.working = 1;
	_robotino_two_d_move_step_as.publishFeedback(feedback);
	ROS_INFO("%s, %d: %s received robotino_two_d_move_step: (%s, %f, %f, %f)", __FILE__, __LINE__, _nh.getNamespace().c_str(), goal_ptr->agent_name.c_str(), goal_ptr->move_x, goal_ptr->move_y, goal_ptr->rotation);

	// Construct the step
	auto agent = std::static_pointer_cast<RobotinoController>(findAgentByName(goal_ptr->agent_name));
	std::shared_ptr<TwoDMoveCapabilityParameters> parameters_ptr = std::make_shared<TwoDMoveCapabilityParameters>(goal_ptr->move_x, goal_ptr->move_y, goal_ptr->rotation);
	//TwoDMoveCapabilityParameters parameters {goal_ptr->move_x, goal_ptr->move_y, goal_ptr->rotation};
	auto step_ptr = std::make_shared<RobotinoTwoDMoveStep>(agent, parameters_ptr);
	
	// Check for preconditions and execute or enqueue the step
	if (goal_ptr->previous_state == "idle") {
		if (agent->is_idle()) {
			std::thread thread(&Step::execute, step_ptr.get());
			thread.detach();
			_currently_executed_steps.push_back(std::move(step_ptr));
		}
		else {
			ROS_INFO("%s, %d: Robotino is not idle. Enqueueing this step until the robot is available.", __FILE__, __LINE__);
			_waiting_steps.push_back(std::move(step_ptr));
		}
	}
	else {
		std::thread thread(&Step::execute, step_ptr.get());
		thread.detach();
		_currently_executed_steps.push_back(std::move(step_ptr));
	}
	// TODO: Wait for completion
	_robotino_two_d_move_step_as.setSucceeded();
}

// This is the callback method for an action server that returns a report on what the workcell is doing right now.
// I stopped working on this when I started working on the UI, so this is far from complete.
void StepServer::statusCallback() {
	factory_floor_manager::status_reportGoalConstPtr goal_ptr;
	factory_floor_manager::status_reportResult result;
	
	if (!_status_report_server.isPreemptRequested()) {
		goal_ptr = _status_report_server.acceptNewGoal();
	}
	// TODO else return?
	
	result.node_name = _nh.getNamespace().c_str();
	for (const auto& controller_ptr : *_controllers_ptr) {
		result.agent_statuses.push_back(controller_ptr->getStatusStr());
	}
	// TODO: Agents, ...
	
	for (const auto& step_ptr : _currently_executed_steps) {
		result.executing_steps.push_back(step_ptr->getDescription());
	}
	for (const auto& step_ptr : _waiting_steps) {
		result.waiting_steps.push_back(step_ptr->getDescription());
	}
	
	_status_report_server.setSucceeded(result);
}

std::shared_ptr<AgentController> StepServer::findAgentByName(const std::string& name) {
	auto result = std::find_if(_controllers_ptr->begin(),
													   _controllers_ptr->end(),
													   [&] (const auto& ptr ) { return ptr->getName() == name; });
	
	if (result != _controllers_ptr->end()) {
		return *result;
	}
	else {
		ROS_ERROR("%s, %d: Agent not found: %s", __FILE__, __LINE__, name.c_str());
		return nullptr;
	}
}

void StepServer::startRobotinoTwoDMoveStepAS() {
	_robotino_two_d_move_step_as.registerGoalCallback(boost::bind(&StepServer::robotinoTwoDMoveGoalCallback, this));
	_robotino_two_d_move_step_as.start();
}

void StepServer::stopRobotinoTwoDMoveStepAS() {
	_robotino_two_d_move_step_as.shutdown();
}

void StepServer::addStep(std::shared_ptr<Step> step_ptr) {
	_currently_executed_steps.push_back(step_ptr);
	// TODO: Can this step currently be executed? If no, we should add it to the waiting steps
	step_ptr->execute();
}
