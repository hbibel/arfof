/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include "robotino_two_d_move_step.h"

RobotinoTwoDMoveStep::RobotinoTwoDMoveStep(std::shared_ptr<RobotinoController> agent_ptr,
																					 std::shared_ptr<TwoDMoveCapabilityParameters> parameters) : _robotino_ptr(agent_ptr)
				, _parameters(parameters) {}

void RobotinoTwoDMoveStep::execute() {
	_robotino_ptr->delegateToComponent(_parameters);
}

std::string RobotinoTwoDMoveStep::getDescription() {
	std::ostringstream ss;
	ss << "Agent: " << _robotino_ptr->getName() << "; Capability: " << _parameters->getCapabilityName()
		<< "; Parameters: " << _parameters->to_x << ", " << _parameters->to_y << ", " << _parameters->to_rotation;
	return ss.str();
}
