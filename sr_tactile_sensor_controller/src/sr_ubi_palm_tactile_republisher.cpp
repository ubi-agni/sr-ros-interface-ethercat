/**
 * @file   ubi_palm_tactile_republisher.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Feb 09 2016
 *
 * @brief  Converts palm tactile data of ubi sensors to tactile state.
 */


#include "sr_tactile_sensor_controller/sr_ubi_palm_tactile_republisher.hpp"
#include "sr_tactile_sensor_controller/sr_tactile_calibration.hpp"
#include <boost/thread/locks.hpp>
#include <string>


using namespace std;

namespace shadowrobot
{
  SrUbiPalmTactileRepublisher::SrUbiPalmTactileRepublisher():
  publish_rate_(DEFAULT_PUBLISH_RATE), prefix_("")
  {
    palm_taxels_.resize(NB_PALM_TAXELS);
    metacarpal_taxels_.resize(NB_METACARPAL_TAXELS);
    
    ros::NodeHandle nh_priv("~");
    // read parameters
    nh_priv.getParam("prefix", prefix_);
    double rate;
    if(nh_priv.getParam("publish_rate", rate))
    {
      publish_rate_= ros::Rate(rate);
    }
    
    
    // prepare the recurrent tactile message
    tactile_msg_.sensors.resize(2);
    tactile_msg_.sensors[0].name = prefix_ + "palm";
    tactile_msg_.sensors[0].values.resize(NB_PALM_TAXELS);
    tactile_msg_.sensors[1].name = prefix_ + "lfmetacarpal";
    tactile_msg_.sensors[1].values.resize(NB_METACARPAL_TAXELS);
    
    // init publisher/subscribers
    init();
  }

  void SrUbiPalmTactileRepublisher::init()
  {
    // calibration maps
    calibration_map_ = read_tactile_calibration(nh);
    
    // initialize publisher
    tactile_pub_ = nh.advertise<tactile_msgs::TactileState>("tactile_palm", 1);
    
    // intialize subscribers
    tac_aux_spi_sub_=nh.subscribe("tactile_aux_spi", 1, // buffer size
                                  &SrUbiPalmTactileRepublisher::palm_aux_spi_cb, this);
    tac_palm_extras_sub_=nh.subscribe("palm_extras", 1, // buffer size
                                      &SrUbiPalmTactileRepublisher::palm_extra_cb, this);
  }

  void SrUbiPalmTactileRepublisher::palm_aux_spi_cb(const sr_robot_msgs::AuxSpiDataConstPtr& msg)
  {
    boost::unique_lock<boost::shared_mutex> lock(mutex_);
    // the aux spi are all relative to palm flesh data.
    for (size_t i = 0; i < NB_AUX_CHANNELS; ++i)
    {
      palm_taxels_[i] = msg->sensors[i];
    }  
  }

  void SrUbiPalmTactileRepublisher::palm_extra_cb(const std_msgs::Float64MultiArrayConstPtr& msg)
  {
    boost::unique_lock<boost::shared_mutex> lock(mutex_);
    
    // the palm extras has one sensor that must be merged to the palm
    // the 3 other sensors belong to the metacarpal 
    // (and hence has another frame, and should be on another ChannelFloat32 message)
    size_t dataoffset=0, datasize=0;
    bool analog_data_found=false;
    for(size_t i = 0; i < msg->layout.dim.size(); ++i)
    {
      // check for the analog data
      if (msg->layout.dim[i].label.find("analog_inputs")!=std::string::npos){
        datasize = msg->layout.dim[i].size;
        analog_data_found=true;
        break;
      }
      // accumulate sizes of not matching sensors
      dataoffset += msg->layout.dim[i].size;
    }

    if (analog_data_found) 
    {
      // all analog values except last
      metacarpal_taxels_.assign(msg->data.begin() + dataoffset, msg->data.begin() + dataoffset + datasize - 1);
    
      // last value belongs to palm, add it at the very end
      palm_taxels_[NB_AUX_CHANNELS] = msg->data[dataoffset + datasize - 1];
    }
  }

  std::vector<float> SrUbiPalmTactileRepublisher::calibrate(std::string sensor_name, std::vector<float> &uncalibrated)
  {
    std::vector<float> calibrated(uncalibrated.size());
    // calibrate the sensor
    calibration_tmp_ = calibration_map_.find(sensor_name);
    if (calibration_tmp_)
    {
      for (unsigned i=0; i<uncalibrated.size(); i++)
      {
          calibrated[i] = calibration_tmp_->compute(static_cast<double> (uncalibrated[i]));
      }
    }
    else
    {
      ROS_WARN_STREAM_ONCE("at least one calibration not found for " << sensor_name << ", storing raw value instead");
      for (unsigned i=0; i<uncalibrated.size(); i++)
      {
          calibrated[i] = static_cast<double> (uncalibrated[i]);
      }
    }
    return calibrated;
  }

  void SrUbiPalmTactileRepublisher::publish()
  {
    {
      boost::shared_lock<boost::shared_mutex> lock(mutex_);
      tactile_msg_.sensors[0].values = this->calibrate(tactile_msg_.sensors[0].name, palm_taxels_);
      tactile_msg_.sensors[1].values = this->calibrate(tactile_msg_.sensors[1].name, metacarpal_taxels_);
      tactile_msg_.header.stamp = ros::Time::now();
      tactile_pub_.publish(tactile_msg_);
    }
    publish_rate_.sleep();
  }

}  // namespace shadowrobot

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sr_ubi_palm_tactile_republisher");

  shadowrobot::SrUbiPalmTactileRepublisher palm_repub;

  while (ros::ok())
  {
    palm_repub.publish();
    ros::spinOnce();
  }

  return 0;
}
