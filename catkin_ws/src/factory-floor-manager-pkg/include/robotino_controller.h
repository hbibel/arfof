/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include "robot_controller.h"
#include "two_d_move_capability.h"
#include "robotino_base_component.h"

/**
 * A controller for the Robotino.
 */
class RobotinoController : public RobotController {
public:
	/**
	 *
	 * @param name             The ROS node name for this controller
	 * @param parentNodeHandle The node handle of the workcell controller in which the robotino resides
	 * @return
	 */
	RobotinoController(std::string &name, const ros::NodeHandle& parentNodeHandle);
	~RobotinoController();
	
	/**
	 * Searches the components of the robotino for a suitable capability. If such a capability is found, it is
	 * executed. If not, a ROS_ERROR is printed.
	 * @param parameters
	 */
	void delegateToComponent(std::shared_ptr<Capability::Parameters> parameters) override;
	
/**
	 * @return true, if the Robotino currently is idle (and connected). False otherwise.
	 */
	bool is_idle();
	
	/**
	 * @return A string describing the current status of the Robotino, i.e. "Idle", "Disconnected" or "Moving".
	 */
	virtual std::string getStatusStr() override;
	
	/**
	 * @return the status of the Robotino.
	 */
	virtual Status getStatus() override;
	
	/**
	 * @return a string describing the type of agent, i.e. "Robotino".
	 */
	virtual std::string getType() override;
	
	/**
	 * Slows down the Robotino to a speed that is safe near humans, i.e. a linear velocity of 0.2 and an angular velocity
	 * of 0.3.
	 */
	void throttleToSafeSpeed() override;
	
	/**
	 * Updates the state of this agent.
	 */
	void updateState() override;

	/**
	 * This method reverts the changes of <throttleToSafeSpeed>"()".
	 */
	void goBackToNormalSpeed();

protected:
	Status _state;
	ros::NodeHandle _nh;

	/**
	 * Sets the maximum linear and angular speed to `linear` and `angular` respectively.
	 * @param linear
	 * @param angular
	 */
	void updateSpeed(double linear, double angular);
	
/**
	 * Checks whether or not a node named `/robotino_node` is online.
	 * \note As the author writes this, this is not sufficient to find out whether or not the robotino actually is
	 * online, since the `/robotino_node` does not get deleted from the ROS network after it has shut down. To check
	 * this, run `roslaunch robotino_node robotino_node.launch` and then terminate the program, then check the output
	 * of `rosnode list`.
	 * @return
	 */
	bool robotinoNodeIsOnline();

private:
	const std::string agent_type = "Robotino";
};