/**
 * @file   sr_tactile_calibration.hpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Feb 17 2016
 *
 * @brief  tactile calibration helpers
 *
 */

#pragma once

#include <ros/ros.h>
#include <sr_robot_lib/sr_joint_motor.hpp>

namespace shadowrobot
{
  shadow_joints::CalibrationMap read_tactile_calibration(ros::NodeHandle &nh);
}
