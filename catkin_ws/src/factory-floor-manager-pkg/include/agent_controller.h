/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "capability.h"

/**
 * @section DESCRIPTION
 * This is the base class for all agent controllers.
 * Agents are humans or robots. A controller provides methods that can be seen as orders that can be given to an agent.
 */
class AgentController {
public:
	enum Status {
		IDLE, MOVING, DISCONNECTED
	};
	/**
	 * Constructor that sets the time to a given value.
	 *
	 * @param name A name given to the agent. It is supposed to be unique, though it is not the duty of the agent
	 * controller to ensure that.
	 */
	AgentController(std::string& name);
	// virtual destructor to make sure AgentController doesn't get instantiated
	virtual ~AgentController() = 0;
	
	/**
	 * @return A string describing the status of the controller, e.g. "Disconnected"
	 */
	virtual std::string getStatusStr() = 0;
	
	/**
	 * @return A string describing the type of the agent, e.g. "Robotino"
	 */
	virtual std::string getType() = 0;
	
	/**
	 * Getter for the current AgentController::Status
	 * @return
	 */
	virtual Status getStatus() = 0;
	
	/**
	 * Updates the internal Status of the controller
	 */
	virtual void updateState() = 0;
	
	/**
	 * The getter function for the node name.
	 * @return The node name
	 */
	const std::string& getName() const;
	
	/**
	 * Iterates over the components and returns their capabilities
	 * @return A vector of pointers to capabilites.
	 */
	virtual const std::vector<std::shared_ptr<Capability>> getCapabilities() = 0;

protected:
	std::string _name; // The node name
};