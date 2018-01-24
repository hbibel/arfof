/*
 * This serves as an example on how to use the status_report_server. The status_report_server enables other nodes
 * to find out what the current status of the FFM is.
 */

#include <factory_floor_manager/robotino_two_d_move_stepAction.h>
#include <actionlib/client/simple_action_client.h>
#include <factory_floor_manager/status_reportAction.h>

typedef actionlib::SimpleActionClient<factory_floor_manager::status_reportAction> Client;

std::string concat_string_array(const std::vector<std::string>& string_vector);

int main(int argc, char** argv) {
	ros::init(argc, argv, "ffm_status_test");
	Client client("/factory_floor_manager/workcell_1/step_server/status_report_server", true); // true -> don't need ros::spin()
	client.waitForServer();
	factory_floor_manager::status_reportGoal goal;
	client.sendGoal(goal);
	client.waitForResult(ros::Duration(100.0));
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Current FFM status:\n%s\n", client.getResult()->node_name.c_str());
		ROS_INFO("Agents:\n%s", concat_string_array(client.getResult()->agent_statuses).c_str());
		ROS_INFO("Steps currently executing:\n%s", concat_string_array(client.getResult()->executing_steps).c_str());
		ROS_INFO("Steps currently in queue:\n%s", concat_string_array(client.getResult()->waiting_steps).c_str());
	}
	return 0;
}

std::string concat_string_array(const std::vector<std::string>& string_vector) {
	std::string result("");
	for (auto s : string_vector) {
		result.append(s).append("\n");
	}
	return result;
}