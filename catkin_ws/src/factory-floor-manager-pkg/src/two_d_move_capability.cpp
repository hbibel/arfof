/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include <ros/ros.h>

#include "two_d_move_capability.h"

TwoDMoveCapability::TwoDMoveCapability(TwoDMoveCapabilityCallback callback) : _callback(callback) {}

TwoDMoveCapability::~TwoDMoveCapability() {}

const std::string& TwoDMoveCapability::getName() {
	static std::string name = "two_d_move";
	return name;
}

void TwoDMoveCapability::perform(std::shared_ptr<Capability::Parameters> p) {
	if (p->getCapabilityName() != getName()) {
		ROS_ERROR("%s, %d: Parameter type mismatch: Expected %s parameters, got %s parameters."
			, __FILE__, __LINE__, getName().c_str(), p->getCapabilityName().c_str());
		return;
	}
	_callback.operator()(std::static_pointer_cast<TwoDMoveCapabilityParameters>(p));
}

const std::string TwoDMoveCapability::Parameters::getCapabilityName() {
	return capability_name;
}

const std::string TwoDMoveCapability::Parameters::capability_name = "two_d_move";