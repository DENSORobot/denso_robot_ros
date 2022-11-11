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

#include "denso_robot_core/denso_robot_core.hpp"
#include "denso_robot_core/denso_controller_rc8.hpp"
#include "denso_robot_core/denso_controller_rc8_cobotta.hpp"
#include "denso_robot_core/denso_controller_rc9.hpp"
#include <boost/thread.hpp>

#include <iostream>

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "denso_robot_core");

//   HRESULT hr;

//   denso_robot_core::DensoRobotCore engine;

//   hr = engine.Initialize();
//   if (FAILED(hr))
//   {
//     ROS_ERROR("Failed to initialize. (%X)", hr);
//     return 1;
//   }
//   else
//   {
//     boost::thread t(boost::bind(&denso_robot_core::DensoRobotCore::Start, &engine));

//     ros::spin();

//     engine.Stop();
//     t.join();

//     return 0;
//   }
// }

namespace denso2
{
DensoRobotCore::DensoRobotCore() : ctrlType_(8), mode_(0), quit_(false)
{
  ctrl_.reset();
}

DensoRobotCore::~DensoRobotCore()
{
}

HRESULT DensoRobotCore::Initialize(int ctrl_type, std::string addr, int port, std::string file_name)
{
  std::string name, filename;
  float ctrl_cycle_msec;
  std::string robot_name;
  ctrlType_ = ctrl_type;
  filename = file_name;
  name = "";
  // if (!node.getParam("controller_name", name))
  // {
  //   name = "";
  // }

  // if (!node.getParam("controller_type", ctrlType_))
  // {
  //   return E_FAIL;
  // }

  // if (!node.getParam("config_file", filename))
  // {
  //   return E_FAIL;
  // }

  // if (!node.getParam("bcap_slave_control_cycle_msec", ctrl_cycle_msec))
  // {
  //   return E_FAIL;
  // }
  // if (!node.getParam("robot_name", robot_name))
  // {
  //   return E_FAIL;
  // }
  switch (ctrlType_)
  {
    case 8:
      if (DensoControllerRC8Cobotta::IsCobotta(robot_name))
      {
        ctrl_ = boost::make_shared<DensoControllerRC8Cobotta>(name, &mode_, addr, port);
      }
      else
      {
        ctrl_ = boost::make_shared<DensoControllerRC8>(name, &mode_, addr, port);
      }
      break;
    case 9:
      ctrl_ = boost::make_shared<DensoControllerRC9>(name, &mode_, addr, port);
      break;
    default:
      // ROS_ERROR("Invalid argument value [controller_type]");
      return E_INVALIDARG;
  }
  HRESULT hr = ctrl_->InitializeBCAP(filename);
  return 0;
}

void DensoRobotCore::Start()
{

  quit_ = false;
  ctrl_->StartService();

  // while (!quit_ && ros::ok())
  // {
  //   ros::spinOnce();
  //   ctrl_->Update();
  //   ros::Rate(1000).sleep();
  // }
}

void DensoRobotCore::Stop()
{
  quit_ = true;
  ctrl_->StopService();
}

HRESULT DensoRobotCore::ChangeMode(int mode, bool service)
{
  ctrl_->StopService();

  DensoRobot_Ptr pRob;
  HRESULT hr = ctrl_->get_Robot(0, &pRob);
  if (SUCCEEDED(hr))
  {
    switch (ctrlType_)
    {
      case 8:
      case 9:
        hr = pRob->ChangeMode(mode);
        break;
      default:
        hr = E_FAIL;
        break;
    }
  }

  mode_ = SUCCEEDED(hr) ? mode : 0;

  if ((mode_ == 0) && service)
  {
    // ros::NodeHandle nd;
    ctrl_->StartService();
  }

  return hr;
}

}  // namespace denso2
