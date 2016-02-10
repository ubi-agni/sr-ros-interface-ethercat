/**
 * @file   sr_ubi_tactile_state_publisher.hpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Feb 08 2016
 *
 * Copyright 2015 Shadow Robot Company Ltd.
 *
 * @brief  Publishes ubi tactile state
 *
 */

/// derived from ImuSensorController  author: Adolfo Rodriguez Tsouroukdissian

#pragma once

#include <sr_tactile_sensor_controller/sr_tactile_sensor_publisher.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <tactile_msgs/TactileState.h>

namespace controller
{

class SrUbiTactileStatePublisher: public SrTactileSensorPublisher
{
public:
  SrUbiTactileStatePublisher(std::vector<tactiles::AllTactileData>* sensors, double publish_rate, ros::NodeHandle nh_prefix, std::string prefix)
        : SrTactileSensorPublisher(sensors, publish_rate, nh_prefix, prefix) {}
  virtual void init();
  virtual void update(const ros::Time& time, const ros::Duration& period);

private:

  typedef realtime_tools::RealtimePublisher<tactile_msgs::TactileState> UbiPublisher;
  typedef boost::shared_ptr<UbiPublisher > UbiPublisherPtr;
  typedef realtime_tools::RealtimePublisher<tactile_msgs::TactileState> MidPublisher;
  typedef boost::shared_ptr<MidPublisher > MidPublisherPtr;
  typedef realtime_tools::RealtimePublisher<tactile_msgs::TactileState> ProxPublisher;
  typedef boost::shared_ptr<ProxPublisher > ProxPublisherPtr;
  UbiPublisherPtr ubi_realtime_pub_;
  MidPublisherPtr mid_realtime_pub_;
  ProxPublisherPtr prox_realtime_pub_;

};

}// namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

