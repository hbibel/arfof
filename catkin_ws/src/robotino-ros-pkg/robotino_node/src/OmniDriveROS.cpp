/*
 * OmniDriveROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "OmniDriveROS.h"
#include <dynamic_reconfigure/server.h>
#include <robotino_node/RobotinoNodeConfig.h>

OmniDriveROS::OmniDriveROS()
{
	cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &OmniDriveROS::cmdVelCallback, this);
}

OmniDriveROS::~OmniDriveROS()
{
	cmd_vel_sub_.shutdown();
}

void OmniDriveROS::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
	double linear_x = msg->linear.x;
	double linear_y = msg->linear.y;
	double angular = msg->angular.z;

	if(dyn_max_linear_vel_ > max_linear_vel_)
	{
		dyn_max_linear_vel_ = max_linear_vel_;
	}

	if(dyn_min_linear_vel_ < min_linear_vel_)
	{
		dyn_min_linear_vel_ = min_linear_vel_;
	}

	if(dyn_max_angular_vel_ > max_angular_vel_)
	{
		dyn_max_angular_vel_ = max_angular_vel_;
	}

	if(dyn_min_angular_vel_ < min_angular_vel_)
	{
		dyn_min_angular_vel_ = min_angular_vel_;
	}

	// before setting velocity, values are compared to dyn max velocity and static max velocity
	

	if ( fabs( linear_x ) > dyn_max_linear_vel_ )
	{
		if( linear_x > 0.0 )
			linear_x = dyn_max_linear_vel_;
		else
			linear_x = -dyn_max_linear_vel_;
	}
	else if( fabs( linear_x ) <  dyn_min_linear_vel_ && fabs( linear_x ) > 0.0 )
	{
		if( linear_x > 0.0 )
			linear_x = dyn_min_linear_vel_;
		else
			linear_x = -dyn_min_linear_vel_;
	}

	if ( fabs( linear_y ) > dyn_max_linear_vel_ )
	{
		if( linear_y > 0.0 )
			linear_y = dyn_max_linear_vel_;
		else
			linear_y = -dyn_max_linear_vel_;
	}
	else if( fabs( linear_y ) <  dyn_min_linear_vel_ && fabs( linear_y ) > 0.0 )
	{
		if( linear_y > 0.0 )
			linear_y = dyn_min_linear_vel_;
		else
			linear_y = -dyn_min_linear_vel_;
	}

	if ( fabs( angular ) > dyn_max_angular_vel_ )
	{
		if( angular > 0.0 )
			angular = dyn_max_angular_vel_;
		else
			angular = -dyn_max_angular_vel_;
	}
	else if( fabs( angular ) <  dyn_min_angular_vel_ && fabs( angular ) > 0.0 )
	{
		if( angular > 0.0 )
			angular = dyn_min_angular_vel_;
		else
			angular = -dyn_min_angular_vel_;
	}

	setVelocity( linear_x, linear_y, angular);
}

void OmniDriveROS::setMaxMin( double max_linear_vel, double min_linear_vel,
		double max_angular_vel, double min_angular_vel )
{
	max_linear_vel_ = max_linear_vel;
	min_linear_vel_ = min_linear_vel;
	max_angular_vel_ = max_angular_vel;
	min_angular_vel_ = min_angular_vel;
}

void OmniDriveROS::setDynMaxMin( double dyn_max_linear_vel, double dyn_min_linear_vel,
		double dyn_max_angular_vel, double dyn_min_angular_vel )
{
	dyn_max_linear_vel_ = dyn_max_linear_vel;
	dyn_min_linear_vel_ = dyn_min_linear_vel;
	dyn_max_angular_vel_ = dyn_max_angular_vel;
	dyn_min_angular_vel_ = dyn_min_angular_vel;
}
