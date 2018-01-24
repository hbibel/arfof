/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "robotino_local_move/LocalMoveFeedback.h"

#include "component.h"
#include "capability.h"

/**
 * @section DESCRIPTION
 * The base component of the robotino can move the robot around. Therefore its only capability is a TwoDMoveCapability.
 */
class RobotinoBaseComponent : public Component {
public:
	/**
	 * @param parentNodeHandle The node handle of the Robotino which this base component is part of.
	 */
	RobotinoBaseComponent(const ros::NodeHandle& parentNodeHandle);
	RobotinoBaseComponent(const RobotinoBaseComponent& other);
	RobotinoBaseComponent& operator=(const RobotinoBaseComponent& other);
	~RobotinoBaseComponent();
	
	/**
	 * Determines whether or not this component has a certian capability. The robotino base component has one capability,
	 * and that is called `two_d_move`.
	 * @param capability_name
	 * @return `true` and this component's TwoDMoveCapability, if `capability_name` is `two_d_move`, false and the empty
	 * capability otherweise.
	 */
	std::pair<bool, std::shared_ptr<Capability>> findCapabilityByName(const std::string& capability_name) override;
	
	/**
	 * This method causes the base component to execute a TwoDMove based on the given parameters
	 * @param parameters
	 */
	void act(std::shared_ptr<TwoDMoveCapabilityParameters> parameters);
	
	/**
	 * @return true, if the component currently is busy, false otherwise.
	 */
	bool isBusy();
	
	virtual const std::vector<std::shared_ptr<Capability>>& getCapabilities() override;
	
	virtual std::string getName() override;

protected:
	enum State {
		IDLE, BUSY
	};
	
	std::shared_ptr<State> _state_ptr;
	ros::NodeHandle _nh;
	ros::Publisher _publisher;
	ros::Subscriber _odometry_subscriber;
	
	double _curr_x, _start_x, _dist_moved, _curr_y, _start_y;
	std::tuple<float, float, float> _previous_feedback;

	void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
	void localMoveFeedbackCallback(const robotino_local_move::LocalMoveFeedbackConstPtr& feedback_ptr);
	void changeState(State new_state);
};