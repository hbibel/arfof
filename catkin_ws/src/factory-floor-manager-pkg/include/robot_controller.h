/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <type_traits>
#include <vector>

#include "agent_controller.h"
#include "capability.h"
#include "component.h"

/**
 * @section DESCRIPTION
 * A robot is an agent which is not human.
 */
class RobotController : public AgentController {
public:
	RobotController(std::string& name) : AgentController(name), _component_ptr_vector() { }
	virtual ~RobotController() = 0;

	/**
	 * Per default, this method prints a ROS error and does nothing else. Classes inheriting this should search their
	 * components for one that supports the needed capability (which should be able to be identified by
	 * `parameters.get_capability_name()`)
	 * and then execute the capability of the suitable componant.
	 * @param parameters An object that is of a subtype of `Capability::Parameters`.
	 */
	virtual void delegateToComponent(std::shared_ptr<Capability::Parameters> parameters);

	/**
	 * A fast moving robot near a human is a hazard. Therefore, there should be a "safe speed" which is set as the
	 * maximum speed once a human is in proximity to the robot.
	 */
	virtual void throttleToSafeSpeed() = 0;

	virtual const std::vector<std::shared_ptr<Capability>> getCapabilities() override;
	
protected:
	std::vector<std::shared_ptr<Component>> _component_ptr_vector;
};