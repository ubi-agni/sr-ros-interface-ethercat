/**
 * @file   sr_ubi_tactile_state_publisher.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Feb 08 2016
 *
 * Copyright 2015 Shadow Robot Company Ltd.
 *
 * @brief  Publishes ubi tactile state.
 *
 */

/// derived from ImuSensorController  author: Adolfo Rodriguez Tsouroukdissian

#include "sr_tactile_sensor_controller/sr_ubi_tactile_state_publisher.hpp"
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <string>

using namespace std;

namespace controller
{
void SrUbiTactileStatePublisher::init()
{
  // realtime publisher
  ubi_realtime_pub_ = UbiPublisherPtr(new realtime_tools::RealtimePublisher<tactile_msgs::TactileState>(nh_prefix_, "tactile_tip", 4));
  mid_realtime_pub_ = MidPublisherPtr(new realtime_tools::RealtimePublisher<tactile_msgs::TactileState>(nh_prefix_, "tactile_mid", 4));
  prox_realtime_pub_ = ProxPublisherPtr(new realtime_tools::RealtimePublisher<tactile_msgs::TactileState>(nh_prefix_, "tactile_prox", 4));
  
  // init data vectors
  ubi_realtime_pub_->msg_.sensors.resize(sensors_->size());
  mid_realtime_pub_->msg_.sensors.resize(sensors_->size());
  prox_realtime_pub_->msg_.sensors.resize(sensors_->size());
  
  // calibration maps
  calibration_map_ = this->read_tactile_calibration();
}

shadow_joints::CalibrationMap SrUbiTactileStatePublisher::read_tactile_calibration()
{
  shadow_joints::CalibrationMap tactile_calibration;

  XmlRpc::XmlRpcValue calib;
  nh_prefix_.getParam("ubi_calibrations", calib);
  if(calib.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    // iterate on all the sensors
    for (int32_t index_cal = 0; index_cal < calib.size(); ++index_cal)
    {
      // check the calibration is well formatted:
      // first sensor name, then calibration table
      if (calib[index_cal][0].getType() == XmlRpc::XmlRpcValue::TypeString  && calib[index_cal][1].getType() == XmlRpc::XmlRpcValue::TypeArray)
      {

        string sensor_name = static_cast<string> (calib[index_cal][0]);
        vector<joint_calibration::Point> calib_table_tmp;

        // now iterates on the calibration table for the current joint
        for (int32_t index_table = 0; index_table < calib[index_cal][1].size(); ++index_table)
        {
          if(calib[index_cal][1][index_table].getType() == XmlRpc::XmlRpcValue::TypeArray)
          {
            // only 2 values per calibration point: raw and calibrated (doubles)
            if(calib[index_cal][1][index_table].size() == 2)
            {
              if(calib[index_cal][1][index_table][0].getType() == XmlRpc::XmlRpcValue::TypeDouble)
              { 
                if(calib[index_cal][1][index_table][1].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {

                  joint_calibration::Point point_tmp;
                  point_tmp.raw_value = static_cast<double> (calib[index_cal][1][index_table][0]);
                  point_tmp.calibrated_value = static_cast<double> (calib[index_cal][1][index_table][1]);
                  calib_table_tmp.push_back(point_tmp);
                  
                }
              }
            }
          }
        }
        ROS_DEBUG_STREAM("calib_table " << shadow_robot::JointCalibration(calib_table_tmp));

        tactile_calibration.insert(sensor_name, boost::shared_ptr<shadow_robot::JointCalibration>(
                                   new shadow_robot::JointCalibration(calib_table_tmp)));
      }
    }
  }
  return tactile_calibration;
}  // end read_joint_calibration

void SrUbiTactileStatePublisher::update(const ros::Time& time, const ros::Duration& period)
{
  bool ubi_published=false;
  
  //init time if not yet initialized
  if (last_publish_time_.toSec() == 0.0)
    last_publish_time_ = time;
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time)
  {
    std::string tactile_prefix[] = {"ff", "mf", "rf", "lf", "th"};
    
    // try to publish
    if (ubi_realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
      ubi_published=true;
      // populate message
      ubi_realtime_pub_->msg_.header.stamp = time;
      //ubi_realtime_pub_->msg_.header.frame_id = prefix_+"distal";
      // data
      for (unsigned i=0; i<sensors_->size(); i++)
      {
        sensor_msgs::ChannelFloat32 sensor_tmp;
        if (i < 5)
          sensor_tmp.name = prefix_ + tactile_prefix[i] + "distal";
        else
        {
          std::stringstream ss;
          ss << prefix_ << "distal" << i;
          sensor_tmp.name = ss.str();
        }
        sensor_tmp.values.resize(sensors_->at(i).ubi0.distal.size());
        
        for (unsigned j=0; j<sensor_tmp.values.size(); j++)
        {
          // calibrate the sensor
          calibration_tmp_ = calibration_map_.find(sensor_tmp.name);
          if (calibration_tmp_)
            sensor_tmp.values[j] = calibration_tmp_->compute(static_cast<double> (sensors_->at(i).ubi0.distal[j]));
          else
          {
            ROS_WARN_STREAM_ONCE("at least one calibration not found for " << sensor_tmp.name << ", storing raw value instead");
            sensor_tmp.values[j] = sensors_->at(i).ubi0.distal[j];
          }
        }
        ubi_realtime_pub_->msg_.sensors[i] = sensor_tmp;
      }
      ubi_realtime_pub_->unlockAndPublish();
    }

    // try to publish
    if (mid_realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      if( !ubi_published)
      {
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
      }
      // populate message
      mid_realtime_pub_->msg_.header.stamp = time;
      //mid_realtime_pub_->msg_.header.frame_id = prefix_+"middle";
      // data
      for (unsigned i=0; i<sensors_->size(); i++)
      {
        sensor_msgs::ChannelFloat32 sensor_tmp;
        if (i < 5)
          sensor_tmp.name = prefix_ + tactile_prefix[i] + "middle";
        else
        {
          std::stringstream ss;
          ss << prefix_ << "middle" << i;
          sensor_tmp.name = ss.str();
        }
        sensor_tmp.values.resize(sensors_->at(i).ubi0.middle.size());
        
        for (unsigned j=0; j<sensor_tmp.values.size(); j++)
        {
          // calibrate the sensor
          calibration_tmp_ = calibration_map_.find(sensor_tmp.name);
          if (calibration_tmp_)
            sensor_tmp.values[j] = calibration_tmp_->compute(static_cast<double> (sensors_->at(i).ubi0.middle[j]));
          else
          {
            ROS_WARN_STREAM_ONCE("at least one calibration not found for " << sensor_tmp.name << ", storing raw value instead");
            sensor_tmp.values[j] = sensors_->at(i).ubi0.middle[j];
          }
        }
        mid_realtime_pub_->msg_.sensors[i] = sensor_tmp;
      }
      mid_realtime_pub_->unlockAndPublish();
    }
    
    // try to publish
    if (prox_realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      if( !ubi_published)
      {
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
      }
      // populate message
      prox_realtime_pub_->msg_.header.stamp = time;
      //prox_realtime_pub_->msg_.header.frame_id = prefix_+"proximal";
      // data
      for (unsigned i=0; i<sensors_->size(); i++)
      {
        sensor_msgs::ChannelFloat32 sensor_tmp;
        if (i < 5)
          sensor_tmp.name = prefix_ + tactile_prefix[i] + "proximal";
        else
        {
          std::stringstream ss;
          ss << prefix_ << "proximal" << i;
          sensor_tmp.name = ss.str();
        }
        sensor_tmp.values.resize(sensors_->at(i).ubi0.proximal.size());
        
        for (unsigned j=0; j<sensor_tmp.values.size(); j++)
        {
          // calibrate the sensor
          calibration_tmp_ = calibration_map_.find(sensor_tmp.name);
          if (calibration_tmp_)
            sensor_tmp.values[j] = calibration_tmp_->compute(static_cast<double> (sensors_->at(i).ubi0.proximal[j]));
          else
          {
            ROS_WARN_STREAM_ONCE("at least one calibration not found for " << sensor_tmp.name << ", storing raw value instead");
            sensor_tmp.values[j] = sensors_->at(i).ubi0.proximal[j];
          }
        }

        prox_realtime_pub_->msg_.sensors[i] = sensor_tmp;
      }
      prox_realtime_pub_->unlockAndPublish();
    }
  }
}
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
