/**
 * @file   sr_ubi_palm_tactile_republisher.hpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Feb 09 2016
 *
 * @brief  Converts palm tactile data of ubi sensors to tactile state.
 */


#ifndef _SR_UBI_PALM_TACTILE_REPUBLISHER_HPP_
#define _SR_UBI_PALM_TACTILE_REPUBLISHER_HPP_

#include <ros/ros.h>

#include <boost/smart_ptr.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <sr_robot_lib/sr_joint_motor.hpp>
#include <sr_utilities/calibration.hpp>
#include <sr_utilities/thread_safe_map.hpp>
#include <sr_robot_msgs/AuxSpiData.h>
#include <std_msgs/Float64MultiArray.h> 
#include <tactile_msgs/TactileState.h> 

#include <string>
#include <vector>

#define NB_PALM_TAXELS 9
#define NB_METACARPAL_TAXELS 3
// only the 8 first data channels are used
#define NB_AUX_CHANNELS 8
#define DEFAULT_PUBLISH_RATE 100


namespace shadowrobot
{
  class SrUbiPalmTactileRepublisher
  {
  public:
    SrUbiPalmTactileRepublisher();

    /**
     * publish the current tactile values
     */
    void publish();
    
    /// A temporary calibration for a given sensor.
    boost::shared_ptr<shadow_robot::JointCalibration> calibration_tmp_;

  private:
  
    ros::NodeHandle nh;
    ros::Rate publish_rate;
    ros::Publisher tactile_pub_;
    ros::Subscriber tac_aux_spi_sub_;
    ros::Subscriber tac_palm_extras_sub_;
    std::string prefix_;
    std::vector<float> palm_taxels_;
    std::vector<float> metacarpal_taxels_;
    tactile_msgs::TactileState tactile_msg_;

  protected:
    mutable boost::shared_mutex mutex_; // multiple reads / one write mutex
    void init();
    void palm_aux_spi_cb(const sr_robot_msgs::AuxSpiDataConstPtr& msg);
    void palm_extra_cb(const std_msgs::Float64MultiArrayConstPtr& msg);
    std::vector<float> calibrate(std::string sensor_name, std::vector<float> &uncalibrated);
    /// The map used to calibrate each tactile.
    shadow_joints::CalibrationMap calibration_map_;
  
  };
}  // namespace shadowrobot


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif //_SR_UBI_PALM_TACTILE_REPUBLISHER_HPP_
