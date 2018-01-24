/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include "robot_controller.h"

void RobotController::delegateToComponent(std::shared_ptr<Capability::Parameters> parameters) {
	ROS_ERROR("This controller does not seem to have the capability to handle parameters of type %s",
						typeid(parameters).name()
	);
}

RobotController::~RobotController() { }

const std::vector<std::shared_ptr<Capability>> RobotController::getCapabilities() {
	std::vector<std::shared_ptr<Capability>> result;
	for (std::shared_ptr<Component> component_ptr : _component_ptr_vector) {
		result.insert(result.end(), component_ptr->getCapabilities().begin(), component_ptr->getCapabilities().end());
	}
	return result;
}