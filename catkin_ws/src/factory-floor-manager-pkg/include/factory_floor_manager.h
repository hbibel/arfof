/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <ros/node_handle.h>

#include "ui/ffm_ui.h"
#include "workcell_controller.h"

class FactoryFloorManager {
public:
	/**
	 * Flags are boolean values in integer fields.
	 */
	struct Flags {
		static const uint8_t Empty = 0;
		static const uint8_t NoUI  = 1;
		// more flags: 2, 4, 8, ... must be powers of 2
		uint8_t value;
		Flags(uint8_t v) : value(v) {};
	};
	
	FactoryFloorManager(int argc, char** argv, Flags flags, ros::NodeHandle&& nh);

/**
 * Starts the factory floor manager. Whether or not it starts with a UI depends on the flags that are set.
 * @return 0, or an error code
 */
	int run();
	
private:
	void state_update_loop();
	
	Flags _flags;
	WorkcellController _workcell_controller;
	std::shared_ptr<FFMUI> _ui_ptr = nullptr;
};