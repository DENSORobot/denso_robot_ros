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

#define S_BUF_FULL (HRESULT)(0x0F200501)
#define E_BUF_FULL (HRESULT)(0x83201483)

namespace denso_robot_core
{
class DensoRobot : public DensoBase
{
  friend class DensoRobotCore;

public:
  enum
  {
    SLVMODE_NONE = 0,
    SLVMODE_POSE_P = 0x0001,
    SLVMODE_POSE_J = 0x0002,
    SLVMODE_POSE_T = 0x0003,
    SLVMODE_POSE = 0x000F,
    SLVMODE_ASYNC = 0x0100,
    SLVMODE_SYNC_WAIT = 0x0200,
  };

  enum
  {
    SENDFMT_NONE = 0,
    SENDFMT_HANDIO = 0x0020,
    SENDFMT_MINIIO = 0x0100,
    SENDFMT_USERIO = 0x0200,
  };

  enum
  {
    RECVFMT_NONE = 0,
    RECVFMT_POSE_P = 0x0001,
    RECVFMT_POSE_J = 0x0002,
    RECVFMT_POSE_T = 0x0003,
    RECVFMT_POSE_PJ = 0x0004,
    RECVFMT_POSE_TJ = 0x0005,
    RECVFMT_POSE = 0x000F,
    RECVFMT_TIME = 0x0010,
    RECVFMT_HANDIO = 0x0020,
    RECVFMT_CURRENT = 0x0040,
    RECVFMT_MINIIO = 0x0100,
    RECVFMT_USERIO = 0x0200,
  };

  enum
  {
    TSFMT_MILLISEC = 0,
    TSFMT_MICROSEC = 1,
  };

  enum
  {
    SLVMODE_TIMEOUT_SYNC = 16,
    SLVMODE_TIMEOUT_ASYNC = 8,
  };

public:
  virtual ~DensoRobot();

  virtual HRESULT InitializeBCAP(XMLElement* xmlElem);

  HRESULT StartService(ros::NodeHandle& node);
  HRESULT StopService();

  bool Update();

  HRESULT get_Variable(const std::string& name, DensoVariable_Ptr* var);

  HRESULT AddVariable(const std::string& name);

  HRESULT ExecTakeArm();
  HRESULT ExecGiveArm();

  void ChangeArmGroup(int number);

  HRESULT ExecMove(int comp, const VARIANT_Ptr& pose, const std::string& option);
  HRESULT ExecDrive(const std::string& name, const VARIANT_Ptr& option);
  HRESULT ExecSpeed(float value);
  HRESULT ExecChange(const std::string& value);
  HRESULT ExecHalt();
  HRESULT ExecCurJnt(std::vector<double>& pose);

  HRESULT ExecSlaveMove(const std::vector<double>& pose, std::vector<double>& joint);

  int get_SendFormat() const;
  void put_SendFormat(int format); /* deprecated */
  int get_RecvFormat() const;
  void put_RecvFormat(int format); /* deprecated */
  int get_TimeFormat() const;
  void put_TimeFormat(int format);
  void set_SendFormat(int format);
  void set_RecvFormat(int format);

  unsigned int get_MiniIO() const;
  void put_MiniIO(unsigned int value);
  unsigned int get_HandIO() const;
  void put_HandIO(unsigned int value);
  void put_SendUserIO(const UserIO& value);
  void put_RecvUserIO(const UserIO& value);
  void get_RecvUserIO(UserIO& value) const;
  void get_Current(std::vector<double>& current) const;
  int get_Timestamp() const;

protected:
  DensoRobot(DensoBase* parent, Service_Vec& service, Handle_Vec& handle, const std::string& name, const int* mode);

  void Callback_ArmGroup(const Int32::ConstPtr& msg);

  virtual HRESULT AddVariable(XMLElement* xmlElem);

  HRESULT CreatePoseData(const PoseData& pose, VARIANT& vnt);

  HRESULT CreateExJoints(const ExJoints& exjoints, VARIANT& vnt);

private:
  virtual HRESULT ChangeMode(int mode);

  HRESULT ExecSlaveMode(const std::string& name, int32_t format, int32_t option = 0);

  HRESULT CreateSendParameter(const std::vector<double>& pose, VARIANT_Ptr& send, const int miniio = 0,
                              const int handio = 0, const int recv_userio_offset = 0, const int recv_userio_size = 0,
                              const int send_userio_offset = 0, const int send_userio_size = 0,
                              const std::vector<uint8_t>& send_userio = std::vector<uint8_t>());

  HRESULT ParseRecvParameter(const VARIANT_Ptr& recv, std::vector<double>& position, std::vector<double>& joint,
                             std::vector<double>& trans, int& miniio, int& handio, int& timestamp,
                             std::vector<uint8_t>& recv_userio, std::vector<double>& current);

private:
  void Callback_MoveString(const MoveStringGoalConstPtr& goal);
  void Callback_MoveValue(const MoveValueGoalConstPtr& goal);
  void Callback_DriveString(const std::string& name, const DriveStringGoalConstPtr& goal);
  void Callback_DriveValue(const std::string& name, const DriveValueGoalConstPtr& goal);
  void Callback_Speed(const Float32::ConstPtr& msg);
  void Callback_Change(const std::string& name, const Int32::ConstPtr& msg);
  void Callback_Cancel();
  void Action_Feedback();

protected:
  DensoVariable_Vec m_vecVar;

  int32_t m_ArmGroup;
  ros::Subscriber m_subArmGroup;

private:
  ros::Subscriber m_subSpeed;
  ros::Subscriber m_subChangeTool;
  ros::Subscriber m_subChangeWork;

  boost::shared_ptr<SimpleActionServer<MoveStringAction> > m_actMoveString;
  boost::shared_ptr<SimpleActionServer<MoveValueAction> > m_actMoveValue;
  boost::shared_ptr<SimpleActionServer<DriveStringAction> > m_actDriveExString;
  boost::shared_ptr<SimpleActionServer<DriveValueAction> > m_actDriveExValue;
  boost::shared_ptr<SimpleActionServer<DriveStringAction> > m_actDriveAExString;
  boost::shared_ptr<SimpleActionServer<DriveValueAction> > m_actDriveAExValue;

  int m_curAct;
  boost::mutex m_mtxAct;

  uint32_t m_memTimeout;
  unsigned int m_memRetry;

  int m_tsfmt, m_timestamp;
  int m_sendfmt, m_send_miniio, m_send_handio;
  int m_recvfmt, m_recv_miniio, m_recv_handio;
  int m_send_userio_offset, m_send_userio_size;
  int m_recv_userio_offset, m_recv_userio_size;
  std::vector<uint8_t> m_send_userio, m_recv_userio;
  std::vector<double> m_position, m_joint, m_trans, m_current;
};

typedef boost::shared_ptr<DensoRobot> DensoRobot_Ptr;
typedef std::vector<DensoRobot_Ptr> DensoRobot_Vec;

}  // namespace denso_robot_core

#endif  // DENSO_ROBOT_H
