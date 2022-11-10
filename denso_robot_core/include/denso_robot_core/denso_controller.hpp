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

#ifndef DENSO_CONTROLLER_HPP__
#define DENSO_CONTROLLER_HPP__

#include "denso_robot_core/denso_base.hpp"
#include "denso_robot_core/denso_robot.hpp"
#include "denso_robot_core/denso_task.hpp"
#include "denso_robot_core/denso_variable.hpp"
#include "denso_robot_core/visibility_control.h"

namespace denso2
{

// using namespace bcap_service;

class DensoController : public DensoBase
{
public:
  static constexpr int BCAP_CONTROLLER_CONNECT_ARGS = 4;
  static constexpr int BCAP_CONTROLLER_EXECUTE_ARGS = 3;

  static constexpr const char* XML_CTRL_NAME = "Controller";

  virtual ~DensoController();

  virtual HRESULT InitializeBCAP(const std::string& filename);

  virtual HRESULT StartService();
  virtual HRESULT StopService();

  virtual bool Update();

  HRESULT get_Robot(int index, DensoRobot_Ptr* robot);
  HRESULT get_Task(const std::string& name, DensoTask_Ptr* task);
  HRESULT get_Variable(const std::string& name, DensoVariable_Ptr* var);

  HRESULT AddVariable(const std::string& name);
  virtual HRESULT ExecClearError();
  virtual HRESULT ExecResetStoState() = 0;
  virtual HRESULT ExecGetCurErrorCount(int& count);
  virtual HRESULT ExecGetCurErrorInfo(int error_index, HRESULT& error_code, std::string& error_message);
  virtual HRESULT ExecGetErrorDescription(HRESULT error_code, std::string& error_description);

  // ros::Duration get_Duration() const
  // {
  //   return duration_;
  // }

protected:
  DensoController(const std::string& name, const int* mode, std::string addr, int port);
  virtual HRESULT AddController() = 0;
  virtual HRESULT AddRobot(XMLElement* xmlElem) = 0;
  virtual HRESULT AddTask(XMLElement* xmlElem);
  virtual HRESULT AddVariable(XMLElement* xmlElem);

protected:
  DensoRobot_Vec vecRobot_;
  DensoTask_Vec vecTask_;
  DensoVariable_Vec vecVar_;
  // ros::Duration duration_;
};

typedef boost::shared_ptr<DensoController> DensoController_Ptr;

}  // namespace denso2

#endif  // DENSO_CONTROLLER_H
