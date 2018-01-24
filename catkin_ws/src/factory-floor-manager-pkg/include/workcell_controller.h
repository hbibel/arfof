/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <vector>

#include "agent_controller.h"
#include "step_server.h"

class WorkcellController {
public:
	/**
	 * @param parent_nh The node handle for the factory floor manager.
	 */
	WorkcellController(ros::NodeHandle& parent_nh);

	/**
	 * Adds the given agent to the workcell.
	 */
	void addAgent(std::shared_ptr<AgentController> agent_ptr);

	const ros::NodeHandle& getNodeHandle() { return _nh; };
	
	/**
	 *
	 * @return a vector of pointers to the agents operated by this workcell controller.
	 */
	const std::vector<std::shared_ptr<AgentController>>& getAgents() { return _agent_controller_ptrs; };
	
	/**
	 *
	 * @return a vector of pointers to steps that are currently executed.
	 */
	const std::vector<std::shared_ptr<Step>>& getExecutingSteps() { return _step_server.getExecutingSteps(); }
	
	void addStep(std::shared_ptr<Step> step_ptr) { _step_server.addStep(step_ptr); }

	/**
	 * Searches for an agent within the worcell.
	 * @param name The node name for the desired agent.
	 * @return The pointer to the desired agent, if it exists within this workcell. Otherwise, it will return a nullptr.
	 */
	std::shared_ptr<AgentController> findAgentByName(const std::string& name);

private:
	ros::NodeHandle _nh;
	std::vector<std::shared_ptr<AgentController>> _agent_controller_ptrs;
	StepServer _step_server;
};