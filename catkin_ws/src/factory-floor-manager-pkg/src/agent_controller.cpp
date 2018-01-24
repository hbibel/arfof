/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include "agent_controller.h"

AgentController::AgentController(std::string& name) : _name(name){}

AgentController::~AgentController() {}

const std::string& AgentController::getName() const {
	return _name;
}

