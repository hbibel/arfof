/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include "factory_floor_manager.h"

int main(int argc, char** argv) {
	ROS_INFO("Starting Factory Floor Manager.");
	ros::init(argc, argv, "factory_floor_manager");
	ros::NodeHandle nh("factory_floor_manager");
	
	FactoryFloorManager::Flags run_flags(FactoryFloorManager::Flags::Empty);
	for (int i = 1; i < argc; ++i) {
		std::string arg = std::string(argv[i]);
		if (arg == "--noui") {
			run_flags.value |= FactoryFloorManager::Flags::NoUI;
		}
	}
	FactoryFloorManager ffm(argc, argv, run_flags, std::move(nh));
	int retcode = ffm.run();
	ros::shutdown();
	return retcode;
}