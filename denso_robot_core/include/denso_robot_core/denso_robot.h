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

#ifndef DENSO_ROBOT_H
#define DENSO_ROBOT_H

#include "denso_robot_core/denso_base.h"
#include "denso_robot_core/denso_variable.h"

#define XML_ROBOT_NAME "Robot"

namespace denso_robot_core {

class DensoRobot : public DensoBase
{
public:
  DensoRobot(DensoBase* parent,
      Service_Vec& service, Handle_Vec& handle,
      const std::string& name, const int* mode);

  virtual ~DensoRobot();

  virtual HRESULT InitializeBCAP(XMLElement *xmlElem);

  virtual HRESULT StartService(ros::NodeHandle& node);
  virtual HRESULT StopService();

  virtual bool Update();

  HRESULT get_Variable(const std::string& name, DensoVariable_Ptr* var);

  HRESULT AddVariable(const std::string& name);

  virtual HRESULT ExecTakeArm() = 0;
  virtual HRESULT ExecGiveArm() = 0;

  void ChangeArmGroup(int number);

protected:
  void Callback_ArmGroup(const Int32::ConstPtr& msg);

  virtual HRESULT AddVariable(XMLElement* xmlElem);

  HRESULT CreatePoseData(
      const PoseData &pose,
      VARIANT &vnt);

  HRESULT CreateExJoints(
      const ExJoints &exjoints,
      VARIANT &vnt);

protected:
  DensoVariable_Vec m_vecVar;

  int32_t m_ArmGroup;
  ros::Subscriber m_subArmGroup;
};

typedef boost::shared_ptr<DensoRobot> DensoRobot_Ptr;
typedef std::vector<DensoRobot_Ptr> DensoRobot_Vec;

}

#endif // DENSO_ROBOT_H
