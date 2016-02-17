/**
 * @file   sr_tactile_calibration.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Feb 17 2016
 *
 * @brief  tactile calibration helpers
 *
 */

#include "sr_tactile_sensor_controller/sr_tactile_calibration.hpp"
#include <string>

using namespace std;

namespace shadowrobot
{

  shadow_joints::CalibrationMap read_tactile_calibration(ros::NodeHandle &nh)
  {
    shadow_joints::CalibrationMap tactile_calibration;

    XmlRpc::XmlRpcValue calib;
    nh.getParam("ubi_calibrations", calib);
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

}
