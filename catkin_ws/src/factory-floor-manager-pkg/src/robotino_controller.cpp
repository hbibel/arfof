/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include <memory>

#include "robotino_controller.h"

#include <dynamic_reconfigure/Reconfigure.h>
#include <thread>

RobotinoController::RobotinoController(std::string &name, const ros::NodeHandle& parentNodeHandle)
				: RobotController(name)
				, _nh(parentNodeHandle, name)
				, _state(DISCONNECTED)
{
	_component_ptr_vector.emplace_back(std::make_shared<RobotinoBaseComponent>(_nh));
}

void RobotinoController::delegateToComponent(std::shared_ptr<Capability::Parameters> parameters) {
	ROS_INFO("%s received command from FFM, sending forward to component", _nh.resolveName("").c_str());
	bool capability_found = false;
	const std::string capability_name = parameters->getCapabilityName();
	for (auto component : _component_ptr_vector) {
		std::pair<bool, std::shared_ptr<Capability>> found_capability = component->findCapabilityByName(capability_name);
		if (found_capability.first) {
			capability_found = true;
			std::shared_ptr<Capability> cabability_ptr = found_capability.second;
			std::thread t(&Capability::perform, cabability_ptr, parameters);
			t.detach();
			break; // exits for loop over _component_ptr_vector
		}
	}
	if (!capability_found) {
		ROS_ERROR("%s, %d: Trying to perform capability that I don't have: %s", __FILE__, __LINE__,
							parameters->getCapabilityName().c_str());
	}
}


void RobotinoController::throttleToSafeSpeed() {
	updateSpeed(0.2, 0.3); // These values are chosen rather arbitrarily.
}

void RobotinoController::goBackToNormalSpeed() {
	updateSpeed(0.6, 1.0);
}

void RobotinoController::updateSpeed(double linear, double angular) {
	dynamic_reconfigure::ReconfigureRequest request;
	dynamic_reconfigure::ReconfigureResponse response;
	dynamic_reconfigure::DoubleParameter dyn_max_linear_vel;
	dynamic_reconfigure::DoubleParameter dyn_min_linear_vel;
	dynamic_reconfigure::DoubleParameter dyn_max_angular_vel;
	dynamic_reconfigure::DoubleParameter dyn_min_angular_vel;
	dynamic_reconfigure::Config config;
	dyn_max_linear_vel.name = "dyn_max_linear_vel";
	dyn_min_linear_vel.name = "dyn_min_linear_vel";
	dyn_max_angular_vel.name = "dyn_max_angular_vel";
	dyn_min_angular_vel.name = "dyn_min_angular_vel";
	dyn_max_linear_vel.value = linear;
	dyn_min_linear_vel.value = 0.01;
	dyn_max_angular_vel.value = angular;
	dyn_min_angular_vel.value = 0.01;
	config.doubles.push_back(dyn_max_linear_vel);
	config.doubles.push_back(dyn_min_linear_vel);
	config.doubles.push_back(dyn_max_angular_vel);
	config.doubles.push_back(dyn_min_angular_vel);
	request.config = config;
	while(!ros::service::call("/robotino_node/set_parameters", request, response)) {
		ROS_INFO("%s could not update parameters. Retrying...", _nh.getNamespace().c_str());
	}
}

RobotinoController::~RobotinoController() {
}

void RobotinoController::updateState() {
	if (robotinoNodeIsOnline()) {
		if (_state == DISCONNECTED) {
			// The node has come online, so initially we set the state to IDLE
			_state = IDLE;
		}
		// Check for current status of the components
		for (auto component_ptr : _component_ptr_vector) {
			// This cast will return a nullptr if component_ptr is not of type RobotinoBaseComponent *
			RobotinoBaseComponent *baseComponent = dynamic_cast<RobotinoBaseComponent *>(component_ptr.get());
			if (nullptr != baseComponent) {
				if (baseComponent->isBusy()) {
					_state = MOVING;
				}
				else if (_state == MOVING) {
					// It seems like the robotino has just stopped moving
					_state = IDLE;
				}
			}
		}
	}
	else {
		_state = DISCONNECTED;
	}
}

bool RobotinoController::robotinoNodeIsOnline() {
	std::vector<std::string> nodes;
	ros::master::getNodes(nodes);
	auto node_it = std::find(nodes.begin(), nodes.end(), "/robotino_node");
	return !(node_it == nodes.end());
}

bool RobotinoController::is_idle() {
	return _state == IDLE;
}

std::string RobotinoController::getStatusStr() {
	std::ostringstream ss;
	ss << getName() << ":\n";
	switch (_state) {
		case IDLE:
			ss << "Idle";
			break;
		case DISCONNECTED:
			ss << "Disconnected";
			break;
		case MOVING:
			ss << "Moving";
			break;
		default:
			ss << "Unknown state";
			break;
	}
	return ss.str();
}

std::string RobotinoController::getType() {
	return agent_type;
}

AgentController::Status RobotinoController::getStatus() {
	return _state;
}
