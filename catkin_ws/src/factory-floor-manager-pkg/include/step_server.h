/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

// Solves an annoying error in CLion:
#pragma ide diagnostic ignored "CannotResolve"

#pragma once

#include <queue>
#include <unordered_map>

#include <actionlib/server/simple_action_server.h>
#include <factory_floor_manager/robotino_two_d_move_stepAction.h>
#include <factory_floor_manager/status_reportAction.h>
#include "robotino_two_d_move_step.h"
#include "step.h"

typedef std::vector<std::shared_ptr<AgentController>>* ControllersPtr;

/**
 * The step server serves as the central point of communication between a workcell and everything around this
 * program. Steps are implemented as ROS actions, and each kind of step gets its own action server that resides
 * within the StepServer. The StepServer also is responsible for enqueuing or rejecting steps that currently can't be
 * executed.
 * Example usage for a RobotinoTwoDMoveStep:
 * \code
 * std::string as_node = "/factory_floor_manager/workcell_1/step_server/robotino_two_d_move_step_server";
 * actionlib::SimpleActionClient<factory_floor_manager::robotino_two_d_move_stepAction> client(as_node, true);
 * client.waitForServer();
 * factory_floor_manager::robotino_two_d_move_stepGoal goal;
 * goal.agent_name = "<insert agent name here>";
 * goal.move_x = ...
 * goal.previous_state = "idle";
 * // Maybe add more preconditions...
 * client.sendGoal(goal);
 * client.waitForResult(ros::Duration(100.0));
 * \endcode
 */
class StepServer {
// Note: This class is currently not fleshed out and un-dynamic. As more agent types get added to this program,
//       functionality for adding and removing agents should be implemented.
public:
	/**
	 * @param parent_nh       The node handle for the workcell controller that owns this StepServer.
	 * @param controller_ptrs A vector of all agents that this StepServer should handle.
	 * @return
	 */
	StepServer(ros::NodeHandle& parent_nh, std::vector<std::shared_ptr<AgentController>>& controller_ptrs);
	
	/**
	 * Starts the action server for RobotinoTwoDMoveSteps
	 */
	void startRobotinoTwoDMoveStepAS();
	
	/**
	 * Stops the action server for RobotinoTwoDMoveSteps
	 */
	void stopRobotinoTwoDMoveStepAS();
	
	void addStep(std::shared_ptr<Step> step_ptr);
	
	/**
	 * Returns a vector containing pointers to the steps that are currently executed.
	 */
	const std::vector<std::shared_ptr<Step>>& getExecutingSteps() { return _currently_executed_steps; }

protected:
	ros::NodeHandle _nh;
	
	actionlib::SimpleActionServer<factory_floor_manager::robotino_two_d_move_stepAction> _robotino_two_d_move_step_as;
	// A status report contains information on the current status of the workcell, i.e. list of agents, steps, ...
	actionlib::SimpleActionServer<factory_floor_manager::status_reportAction> _status_report_server;
	ControllersPtr _controllers_ptr;
	std::vector<std::shared_ptr<Step>> _waiting_steps;
	std::vector<std::shared_ptr<Step>> _currently_executed_steps;

	void robotinoTwoDMoveGoalCallback();
	void statusCallback();
	
	std::shared_ptr<AgentController> findAgentByName(const std::string& name);
};