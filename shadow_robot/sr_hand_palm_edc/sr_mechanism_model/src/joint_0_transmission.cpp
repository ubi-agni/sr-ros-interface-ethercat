/**
 * @file   joint_0_transmission.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Jun 28 11:35:05 2011
 *
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief This is the implementation of the transmission for the joint 0s.
 * We need a specific transmission which takes into account that 2 joints
 * are actuated with only one actuator.
 *
 *
 */

#include "sr_mechanism_model/joint_0_transmission.hpp"

#include <math.h>
#include <pluginlib/class_list_macros.h>
#include "pr2_mechanism_model/robot.h"
#include "pr2_mechanism_model/simple_transmission.h"

using namespace pr2_hardware_interface;

PLUGINLIB_DECLARE_CLASS(sr_mechanism_model, J0Transmission,
                        sr_mechanism_model::J0Transmission,
                        pr2_mechanism_model::Transmission)


namespace sr_mechanism_model
{
  bool J0Transmission::initXml(TiXmlElement *elt, pr2_mechanism_model::Robot *robot)
  {
    const char *name = elt->Attribute("name");
    name_ = name ? name : "";

    TiXmlElement *jel = elt->FirstChildElement("joint1");
    init_joint(jel, robot);
    TiXmlElement *jel2 = elt->FirstChildElement("joint2");
    init_joint(jel2, robot);

    TiXmlElement *ael = elt->FirstChildElement("actuator");
    const char *actuator_name = ael ? ael->Attribute("name") : NULL;
    Actuator *a;
    if (!actuator_name || (a = robot->getActuator(actuator_name)) == NULL )
    {
      ROS_ERROR("J0Transmission could not find actuator named \"%s\"", actuator_name);
      return false;
    }
    a->command_.enable_ = true;
    actuator_names_.push_back(actuator_name);

    mechanical_reduction_ = atof(elt->FirstChildElement("mechanicalReduction")->GetText());

    return true;
  }

  bool J0Transmission::initXml(TiXmlElement *elt)
  {
    const char *name = elt->Attribute("name");
    name_ = name ? name : "";

    TiXmlElement *jel = elt->FirstChildElement("joint1");
    init_joint(jel, NULL);
    TiXmlElement *jel2 = elt->FirstChildElement("joint2");
    init_joint(jel2, NULL);


    TiXmlElement *ael = elt->FirstChildElement("actuator");
    const char *actuator_name = ael ? ael->Attribute("name") : NULL;
    if (!actuator_name)
    {
      ROS_ERROR("J0Transmission could not find actuator named \"%s\"", actuator_name);
      return false;
    }
    actuator_names_.push_back(actuator_name);

    mechanical_reduction_ = atof(elt->FirstChildElement("mechanicalReduction")->GetText());

    return true;
  }

  bool J0Transmission::init_joint(TiXmlElement *jel, pr2_mechanism_model::Robot *robot)
  {
    const char *joint_name = jel ? jel->Attribute("name") : NULL;
    if (!joint_name)
    {
      ROS_ERROR("J0Transmission did not specify joint name");
      return false;
    }

    if( robot != NULL )
    {
      const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);
      if (!joint)
      {
        ROS_ERROR("J0Transmission could not find joint named \"%s\"", joint_name);
        return false;
      }
    }
    joint_names_.push_back(joint_name);

    return true;
  }

  void J0Transmission::propagatePosition(
    std::vector<Actuator*>& as, std::vector<pr2_mechanism_model::JointState*>& js)
  {
    assert(as.size() == 1);
    assert(js.size() == 2);


    //ROS_ERROR_STREAM( "READING pos " << as[0]->state_.position_ );// = js[0]->position_ + js[1]->position_;

    js[0]->position_ = (as[0]->state_.position_ / mechanical_reduction_) + js[0]->reference_position_;
    js[1]->position_ = (as[0]->state_.position_ / mechanical_reduction_) + js[1]->reference_position_;

    js[0]->velocity_ = as[0]->state_.velocity_ / mechanical_reduction_;
    js[1]->velocity_ = as[0]->state_.velocity_ / mechanical_reduction_;

    js[0]->measured_effort_ = as[0]->state_.last_measured_effort_ * mechanical_reduction_;
    js[1]->measured_effort_ = as[0]->state_.last_measured_effort_ * mechanical_reduction_;
  }

  void J0Transmission::propagatePositionBackwards(
    std::vector<pr2_mechanism_model::JointState*>& js, std::vector<Actuator*>& as)
  {
    ROS_ERROR("propagate pos backward");

    assert(as.size() == 1);
    assert(js.size() == 2);
/*
  as[0]->state_.position_ = (js[0]->position_ - js[0]->reference_position_) * mechanical_reduction_;
  as[0]->state_.velocity_ = js[0]->velocity_ * mechanical_reduction_;
*/
    as[0]->state_.last_measured_effort_ = (js[0]->measured_effort_ + js[1]->measured_effort_) / mechanical_reduction_;

    // Update the timing (making sure it's initialized).
    if (! simulated_actuator_timestamp_initialized_)
    {
      // Set the time stamp to zero (it is measured relative to the start time).
      as[0]->state_.sample_timestamp_ = ros::Duration(0);

      // Try to set the start time.  Only then do we claim initialized.
      if (ros::isStarted())
      {
        simulated_actuator_start_time_ = ros::Time::now();
        simulated_actuator_timestamp_initialized_ = true;
      }
    }
    else
    {
      // Measure the time stamp relative to the start time.
      as[0]->state_.sample_timestamp_ = ros::Time::now() - simulated_actuator_start_time_;
    }
    // Set the historical (double) timestamp accordingly.
    as[0]->state_.timestamp_ = as[0]->state_.sample_timestamp_.toSec();

    // simulate calibration sensors by filling out actuator states
    this->joint_calibration_simulator_.simulateJointCalibration(js[0],as[0]);
  }

  void J0Transmission::propagateEffort(
    std::vector<pr2_mechanism_model::JointState*>& js, std::vector<Actuator*>& as)
  {
    assert(as.size() == 1);
    assert(js.size() == 2);
    as[0]->command_.enable_ = true;
    as[0]->command_.effort_ = (js[0]->commanded_effort_ + js[1]->commanded_effort_) / mechanical_reduction_;
  }

  void J0Transmission::propagateEffortBackwards(
    std::vector<Actuator*>& as, std::vector<pr2_mechanism_model::JointState*>& js)
  {
    ROS_ERROR("propagate effort backward");

    assert(as.size() == 1);
    assert(js.size() == 2);
    js[0]->commanded_effort_ = as[0]->command_.effort_ * mechanical_reduction_;
    js[1]->commanded_effort_ = as[0]->command_.effort_ * mechanical_reduction_;
  }

} //end namespace sr_mechanism_model

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
