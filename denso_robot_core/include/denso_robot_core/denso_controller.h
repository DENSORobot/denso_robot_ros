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

#ifndef DENSO_CONTROLLER_H
#define DENSO_CONTROLLER_H

#include "denso_robot_core/denso_base.h"
#include "denso_robot_core/denso_robot.h"
#include "denso_robot_core/denso_task.h"
#include "denso_robot_core/denso_variable.h"

#define XML_CTRL_NAME "Controller"

namespace denso_robot_core
{
class DensoController : public DensoBase
{
public:
  virtual ~DensoController();

  virtual HRESULT InitializeBCAP(const std::string& filename);

  virtual HRESULT StartService(ros::NodeHandle& node);
  virtual HRESULT StopService();

  virtual bool Update();

  HRESULT get_Robot(int index, DensoRobot_Ptr* robot);
  HRESULT get_Task(const std::string& name, DensoTask_Ptr* task);
  HRESULT get_Variable(const std::string& name, DensoVariable_Ptr* var);

  HRESULT AddVariable(const std::string& name);
  virtual HRESULT ExecClearError();
  virtual HRESULT ExecGetCurErrorCount(int& count);
  virtual HRESULT ExecGetCurErrorInfo(int error_index, HRESULT& error_code, std::string& error_message);
  virtual HRESULT ExecGetErrorDescription(HRESULT error_code, std::string& error_description);

  ros::Duration get_Duration() const
  {
    return m_duration;
  }

protected:
  DensoController(const std::string& name, const int* mode, const ros::Duration dt);
  virtual HRESULT AddController() = 0;
  virtual HRESULT AddRobot(XMLElement* xmlElem) = 0;
  virtual HRESULT AddTask(XMLElement* xmlElem);
  virtual HRESULT AddVariable(XMLElement* xmlElem);

protected:
  DensoRobot_Vec m_vecRobot;
  DensoTask_Vec m_vecTask;
  DensoVariable_Vec m_vecVar;
  ros::Duration m_duration;
};

typedef boost::shared_ptr<DensoController> DensoController_Ptr;

}  // namespace denso_robot_core

#endif  // DENSO_CONTROLLER_H
