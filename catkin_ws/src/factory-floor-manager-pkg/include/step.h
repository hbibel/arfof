/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <memory>

#include "agent_controller.h"
#include "capability.h"

class Step {
public:
	virtual void execute() = 0;
	virtual std::string getDescription() = 0;

protected:
	enum State {
		RUNNING, SUCCEEDED, FAILED
	};
};