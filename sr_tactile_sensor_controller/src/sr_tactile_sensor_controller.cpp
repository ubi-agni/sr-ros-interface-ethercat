/**
* @file   sr_tactile_sensor_controller.cpp
* @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
* @date   Aug 22 2014
*
* Copyright 2014 University of Bielefeld
*
* @brief  Generic controller for tactile sensor data publishing.
*
*/

///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of PAL Robotics S.L. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////
/// derived from ImuSensorController  author: Adolfo Rodriguez Tsouroukdissian

#include "sr_tactile_sensor_controller/sr_tactile_sensor_controller.hpp"
#include <pluginlib/class_list_macros.h>
#include <sr_tactile_sensor_controller/sr_pst_tactile_sensor_publisher.hpp>
#include <sr_tactile_sensor_controller/sr_biotac_tactile_sensor_publisher.hpp>
#include <sr_tactile_sensor_controller/sr_ubi_tactile_sensor_publisher.hpp>
#include <sr_tactile_sensor_controller/sr_ubi_tactile_state_publisher.hpp>
#include <string>


namespace controller
{
SrTactileSensorController::SrTactileSensorController()
  : initialized_(false), sensors_(NULL)
{}

bool SrTactileSensorController::init(ros_ethercat_model::RobotStateInterface* hw,
                                     ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
{
  ROS_ASSERT(hw);

  ros_ethercat_model::RobotState* robot_state;
  std::string robot_state_name;
  controller_nh.param<std::string>("robot_state_name", robot_state_name, "unique_robot_hw");

  bool use_ns = true;
  ros::NodeHandle nh_priv("~");
  std::string serial_id;
  std::string hand_id;
  std::string joint_prefix;
  std::map<std::string, std::string> joint_prefix_mapping;
  
  try
  {
    robot_state = hw->getHandle(robot_state_name).getState();
  }
  catch(const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("Could not find robot state: " << robot_state_name << " Not loading the controller. " << e.what());
    return false;
  }

  if (!nh_priv.getParam("use_ns", use_ns))
  {
    ROS_INFO("Private parameter 'use_ns' not set, default is using namespace");
  }
  
  if (!controller_nh.getParam("prefix", joint_prefix))
  {
    ROS_ERROR("Parameter 'prefix' not set");
    return false;
  }
  
  if (!joint_prefix.empty())
  {
    prefix_ = joint_prefix + "_";
 
    // find the serial that matches the joint_prefix
    ros::param::get("/hand/joint_prefix", joint_prefix_mapping);
    for (map<string, string>::const_iterator prefix_iter = joint_prefix_mapping.begin();
           prefix_iter != joint_prefix_mapping.end(); ++prefix_iter)
    {
      if (prefix_ == prefix_iter->second)
        serial_id = prefix_iter->first;
    }
    
    // find the mapping for this serial
    if (!serial_id.empty())
    {
      if (!root_nh.getParam("/hand/mapping/"+serial_id , hand_id))
      {
        ROS_INFO_STREAM("Mapping not set for serial_id " << serial_id << ", using prefix as hand_id");
        hand_id = joint_prefix;
      }
    }
    else
    {
      ROS_INFO_STREAM("Cannot find the serial corresponding to prefix " << prefix_);
      //TODO: normally the sensors handlers should be found via serial_id, and hence fail here
      // currently they are found via joint_prefix in one of the actuators, so we don't care yet.
      hand_id = joint_prefix;
    }
  }

  if (!hand_id.empty())
  {
    if(use_ns)
      nh_prefix_ = ros::NodeHandle(root_nh, hand_id);
    else
      nh_prefix_ = ros::NodeHandle(root_nh);

    if (prefix_.empty())
    {
      prefix_ = hand_id + "_";
    }
  }
  else
  {
    nh_prefix_ = ros::NodeHandle(root_nh);
  }

  // get all sensors from the hardware interface
  // apparently all the actuators have the tactile data copied in, so take the first one.
  sr_actuator::SrMotorActuator* motor_actuator = static_cast<sr_actuator::SrMotorActuator*>(
    robot_state->getActuator(prefix_+"FFJ0"));
  if (motor_actuator)
  {
    sensors_ = motor_actuator->motor_state_.tactiles_;

    // get publishing period
    if (!controller_nh.getParam("publish_rate", publish_rate_))
    {
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
    }

    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Could not find the " << prefix_ << "FFJ0 actuator");
    return false;
  }
}

void SrTactileSensorController::update(const ros::Time& time, const ros::Duration& period)
{
  if (!initialized_)
  {
    if (sensors_)
    {
      if (!sensors_->empty())
      {
        if (!sensors_->at(0).type.empty())
        {
          if (sensors_->at(0).type == "pst")
          {
            boost::shared_ptr<SrTactileSensorPublisher> sensor_publisher(new SrPSTTactileSensorPublisher(sensors_, publish_rate_, nh_prefix_, prefix_));
            sensor_publishers_.push_back(sensor_publisher);
          }
          else if (sensors_->at(0).type == "biotac")
          {
            boost::shared_ptr<SrTactileSensorPublisher> sensor_publisher(new SrBiotacTactileSensorPublisher(sensors_, publish_rate_, nh_prefix_, prefix_));
            sensor_publishers_.push_back(sensor_publisher);
          }
          else if (sensors_->at(0).type == "ubi")
          {
            boost::shared_ptr<SrTactileSensorPublisher> sensor_publisher(new SrUbiTactileSensorPublisher(sensors_, publish_rate_, nh_prefix_, prefix_));
            sensor_publishers_.push_back(sensor_publisher);
            sensor_publisher.reset(new SrUbiTactileStatePublisher(sensors_, publish_rate_, nh_prefix_, prefix_));
            sensor_publishers_.push_back(sensor_publisher);
          }
          else
          {
            ROS_FATAL_STREAM("Unknown tactile sensor type: " << sensors_->at(0).type);
          }

          for (size_t i=0; i < sensor_publishers_.size(); ++i)
          {
            // initialize pusblisher and starting time
            sensor_publishers_[i]->init(time);
          }
          initialized_ = true;
        }
      }
    }
  }
  else
  {
    for (size_t i=0; i < sensor_publishers_.size(); ++i)
    {
      sensor_publishers_[i]->update(time, period);
    }
  }
}

void SrTactileSensorController::starting(const ros::Time& time)
{
}

void SrTactileSensorController::stopping(const ros::Time& time)
{
  // remove initialized flag to permit data type change and time resetting
  initialized_ = false;
}
}  // namespace controller


PLUGINLIB_EXPORT_CLASS(controller::SrTactileSensorController, controller_interface::ControllerBase)

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
