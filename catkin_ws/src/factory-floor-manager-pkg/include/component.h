/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <ros/ros.h>

#include "capability.h"

/**
 * @section DESCRIPTION
 * A component is a part of an agent. A component can have several capabilities. An agent's capabilities is the sum
 * of all of its components' abilities.
 */
class Component {
public:
	Component() : _capabilities_vector() {}

	/**
	 * Searches the capabilities of the component for the given capability.
	 * @param capability_name The name of the desired capability, e.g. "two_d_move"
	 * @return A pair of a boolean and a capability. If the capability is found, the first value is true, and the
	 *         second one is the corresponding capability.
	 *         If the capability has not been found for this component, the first value is false and the second value
	 *         is the empty capability (which usually can be discarded).
	 */
	virtual std::pair<bool, std::shared_ptr<Capability>> findCapabilityByName(const std::string& capability_name) = 0;
	
	/**
	 * @return The capabilities of this component
	 */
	virtual const std::vector<std::shared_ptr<Capability>>& getCapabilities() = 0;
	
	virtual std::string getName() = 0;

protected:
	std::vector<std::shared_ptr<Capability>> _capabilities_vector;
};