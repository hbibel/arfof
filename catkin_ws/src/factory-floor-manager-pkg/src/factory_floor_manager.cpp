/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include "factory_floor_manager.h"

FactoryFloorManager::FactoryFloorManager(int argc, char **argv, FactoryFloorManager::Flags flags,
																				 ros::NodeHandle&& nh)
				: _workcell_controller(nh), _flags { flags.value } {
	if (!(_flags.value & Flags::NoUI)) {
		// The UI only is constructed if the NoUI flag is not set. Otherwise all dereferencings of _ui_ptr will result
		// in a segmentation fault error.
		_ui_ptr = std::make_shared<FFMUI>(argc, argv, &_workcell_controller);
	}
}

void FactoryFloorManager::state_update_loop() {
	// This method runs in its own thread and calls the state update method of all agents.
	ros::Rate rate(10.0);
	rate.sleep();
	while (ros::ok()) {
		for (auto agent_ptr : _workcell_controller.getAgents()) {
			agent_ptr->updateState();
		}
		if (!(_flags.value & Flags::NoUI)) { // if the NoUI flag is not set
			_ui_ptr->updateAgentList();
		}
		rate.sleep();
	}
}

int FactoryFloorManager::run() {
	ros::AsyncSpinner spinner(4);
	spinner.start();
	
	std::thread t(&FactoryFloorManager::state_update_loop, this);
	t.detach();
	
	int retcode;
	if (!(_flags.value & Flags::NoUI)) { // if the NoUI flag is not set
		retcode = _ui_ptr->exec();
	}
	else {
		// Give ros time to set up the node handles
		ros::Rate rate(0.5);
		rate.sleep();
		
		ROS_INFO("%s : %d : Factory Floor Manager up and ready.", __FILE__, __LINE__);
		rate = ros::Rate (10.0);
		while(ros::ok()) {
			rate.sleep();
		}
		retcode = 0;
	}
	return retcode;
}
