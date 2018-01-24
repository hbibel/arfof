/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include "robotino_controller.h"
#include "step.h"

/**
 * This kind of step defines how a Robotino should move. Since Robotinos can't fly, the movement is two-dimensional.
 * If the angle of the movement becomes important, it should be extended by another dimension.
 */
class RobotinoTwoDMoveStep : public Step {
public:
	RobotinoTwoDMoveStep(std::shared_ptr<RobotinoController> agent_ptr, std::shared_ptr<TwoDMoveCapabilityParameters> parameters);
	
	/**
	 * Tell the agent to execute the step.
	 */
	void execute() override;
	
	/**
	 * @return a string describing the agent, the required capability and the parameters for this step.
	 */
	virtual std::string getDescription() override;

protected:
	std::shared_ptr<RobotinoController> _robotino_ptr;
	std::shared_ptr<TwoDMoveCapabilityParameters> _parameters;
};