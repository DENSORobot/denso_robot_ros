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

#include "denso_robot_core/denso_robot_core.h"
#include "denso_robot_core/denso_controller_rc8.h"
#include <boost/thread.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "denso_robot_core");

  HRESULT hr;

  denso_robot_core::DensoRobotCore engine;

  hr = engine.Initialize();
  if(FAILED(hr)){
    ROS_ERROR("Failed to initialize. (%X)", hr);
    return 1;
  } else {
    boost::thread t(boost::bind(&denso_robot_core::DensoRobotCore::Start, &engine));

    ros::spin();

    engine.Stop();
    t.join();

    return 0;
  }
}

namespace denso_robot_core {

DensoRobotCore::DensoRobotCore()
  : m_ctrlType(0), m_mode(0), m_quit(false)
{
  m_ctrl.reset();
}

DensoRobotCore::~DensoRobotCore()
{

}

HRESULT DensoRobotCore::Initialize()
{
  ros::NodeHandle node;
  std::string name, filename;

  if(!node.getParam("controller_name", name)){
    name = "";
  }

  if(!node.getParam("controller_type", m_ctrlType)){
    return E_FAIL;
  }

  if(!node.getParam("config_file", filename)){
    return E_FAIL;
  }

  switch(m_ctrlType){
    case 8:
      m_ctrl = boost::make_shared<DensoControllerRC8>(name, &m_mode);
      break;
    default:
      return E_FAIL;
  }

  return m_ctrl->InitializeBCAP(filename);
}

void DensoRobotCore::Start()
{
  ros::NodeHandle nd;

  m_quit = false;
  m_ctrl->StartService(nd);

  while(!m_quit && ros::ok()){
    ros::spinOnce();
    m_ctrl->Update();
    ros::Rate(1000).sleep();
  }
}

void DensoRobotCore::Stop()
{
  m_quit = true;
  m_ctrl->StopService();
}

HRESULT DensoRobotCore::ChangeMode(int mode, bool service)
{
  m_ctrl->StopService();

  DensoRobot_Ptr pRob;
  HRESULT hr = m_ctrl->get_Robot(0, &pRob);
  if(SUCCEEDED(hr)) {
    switch(m_ctrlType) {
      case 8:
        hr = boost::dynamic_pointer_cast<DensoRobotRC8>(pRob)->ChangeMode(mode);
        break;
      default:
        hr = E_FAIL;
        break;
    }
  }

  m_mode = SUCCEEDED(hr) ? mode : 0;

  if((m_mode == 0) && service) {
    ros::NodeHandle nd;
    m_ctrl->StartService(nd);
  }

  return hr;
}

}
