/**
 * Software License Agreement (MIT License)
 *
 * @copyright Copyright (c) 2015 DENSO WAVE INCORPORATED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "denso_robot_control/denso_robot_hw.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "denso_robot_control");
  ros::NodeHandle nh;

  denso_robot_control::DensoRobotHW drobo;
  HRESULT hr = drobo.Initialize();
  if(SUCCEEDED(hr)) {
      controller_manager::ControllerManager cm(&drobo, nh);

      ros::Rate rate(1.0 / drobo.getPeriod().toSec());
      ros::AsyncSpinner spinner(1);
      spinner.start();

      ros::Time start = drobo.getTime();
      while(ros::ok())
      {
        ros::Time now = drobo.getTime();
        ros::Duration dt = drobo.getPeriod();

        drobo.read(now, dt);

        cm.update(now, dt);

        ros::Duration diff = now - start;
        if(diff.toSec() > 5) {
          drobo.write(now, dt);
        } else {
          rate.sleep();
          continue;
        }

        if (!drobo.is_SlaveSyncMode())
        {
          rate.sleep();
        }
      }
      spinner.stop();
  }

  return 0;
}
