/**
 * @file   motor_updater.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, <contact@shadowrobot.com>
 * @date   Tue Jun  7 09:15:21 2011
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
*
 * @brief  This contains a class used to determin which data we should ask the motor for,
 * depending on the config we're using.
 *
 *
 */

#ifndef _MOTOR_UPDATER_HPP_
#define _MOTOR_UPDATER_HPP_

#include <vector>


#include <sr_edc_ethercat_drivers/types_for_external.h>
extern "C" {
  #include "external/0220_palm_edc/0220_palm_edc_ethercat_protocol.h"
}

namespace motor_updater
{
  struct UpdateConfig
  {
    int what_to_update;
    int when_to_update;
    bool is_important;
  };

  class MotorUpdater
  {
  public:
    MotorUpdater();
    ~MotorUpdater();

    void build_update_motor_command(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND* command);

  private:
    bool even_motors;
    int counter;

    std::vector<UpdateConfig> update_configs_vector;
  };
}


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif