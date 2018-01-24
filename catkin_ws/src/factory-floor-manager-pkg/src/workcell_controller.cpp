/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include "workcell_controller.h"
#include "agent_controller.h"

WorkcellController::WorkcellController(ros::NodeHandle& parent_nh)
		: _nh(parent_nh, "workcell_1")
		, _agent_controller_ptrs{}
		, _step_server{_nh, _agent_controller_ptrs} {}

void WorkcellController::addAgent(std::shared_ptr<AgentController> agent_ptr) {
	if (agent_ptr->getType() == "Robotino") {
		_step_server.startRobotinoTwoDMoveStepAS();
	}
	_agent_controller_ptrs.push_back(std::move(agent_ptr));
}

std::shared_ptr<AgentController> WorkcellController::findAgentByName(const std::string& name) {
	for (auto& agent_controller_ptr : _agent_controller_ptrs) {
		if (agent_controller_ptr->getName() == name) {
			return agent_controller_ptr;
		}
	}
	ROS_ERROR("%s, %d: Agent %s not found in %s", __FILE__, __LINE__, name.c_str(), _nh.getNamespace().c_str());
	return nullptr;
}



