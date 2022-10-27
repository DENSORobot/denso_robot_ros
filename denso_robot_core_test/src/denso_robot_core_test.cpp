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
#include <actionlib/client/simple_action_client.h>

#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>

#include "bcap_core/dn_common.h"

// Message
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include "denso_robot_core/Joints.h"
#include "denso_robot_core/ExJoints.h"
#include "denso_robot_core/PoseData.h"
#include "denso_robot_core/MoveStringAction.h"
#include "denso_robot_core/MoveValueAction.h"
#include "denso_robot_core/DriveStringAction.h"
#include "denso_robot_core/DriveValueAction.h"

using namespace std_msgs;
using namespace actionlib;
using namespace denso_robot_core;

#define MESSAGE_QUEUE   (100)
#define DEFAULT_TIMEOUT (0.0)

enum {
  WRITE_VARIABLE = 1,
  WRITE_ID,
  ARM_GROUP,
  MOVE_STRING,
  MOVE_VALUE,
  DRIVE_STRING,
  DRIVE_VALUE,
  SPEED,
  CHANGE_TOOLWORK,
};

volatile bool bConn;

void Connected(const ros::SingleSubscriberPublisher& pub)
{
  bConn = true;
}

void WaitForConnect()
{
  ros::NodeHandle nh;
  while(!bConn && ros::ok()) {
    ros::spinOnce();
  }
}

volatile bool bPrint;

void PrintFeedback(const std::vector<double>& pose)
{
  std::cout << ros::Time::now() << " ";

  for(int i = 0; i <pose.size(); i++) {
    std::cout << pose[i];
    if(i != pose.size() - 1) {
      std::cout << ", ";
    } else {
      std::cout << std::endl;
    }
  }
}

void PrintThread()
{
  ros::NodeHandle nh;
  while(!bPrint && ros::ok()) {
    ros::spinOnce();
  }
}

void ReadLine(std::string &str, std::string erase = "")
{
  char cline[1024];
  std::cin.getline(cline, sizeof(cline));
  std::getline(std::cin, str);
  std::cin.putback('\n');

  if(!erase.compare("")) {
    size_t c;
    while((c = str.find_first_of(erase)) != std::string::npos) {
      str.erase(c,1);
    }
  }
}

void WriteVariable(const std::string& name)
{
  ros::NodeHandle nh;
  ros::Publisher  pub;

  int i, vt;
  std::string strTmp;
  std::vector<std::string> vecStr;

  Int32  ival; Float32  fval; Float64 dval;
  String sval; Bool bval;
  Float32MultiArray faryval;
  Float64MultiArray daryval;

  std::cout << "Please input variable type: ";
  std::cin >> vt;

  std::cout << "Please input value: ";
  ReadLine(strTmp, " ");
  boost::split(vecStr, strTmp, boost::is_any_of(","));

  switch(vt) {
    case VT_I4:
      pub = nh.advertise<Int32>(name, MESSAGE_QUEUE,
          (ros::SubscriberStatusCallback)Connected);
      ival.data = atoi(vecStr.front().c_str());
      WaitForConnect();
      pub.publish(ival);
      break;
    case VT_R4:
      pub = nh.advertise<Float32>(name, MESSAGE_QUEUE,
          (ros::SubscriberStatusCallback)Connected);
      fval.data = atof(vecStr.front().c_str());
      WaitForConnect();
      pub.publish(fval);
      break;
    case VT_R8:
      pub = nh.advertise<Float64>(name, MESSAGE_QUEUE,
          (ros::SubscriberStatusCallback)Connected);
      dval.data = atof(vecStr.front().c_str());
      WaitForConnect();
      pub.publish(dval);
      break;
    case VT_BSTR:
      pub = nh.advertise<String>(name, MESSAGE_QUEUE,
          (ros::SubscriberStatusCallback)Connected);
      sval.data = vecStr.front();
      WaitForConnect();
      pub.publish(sval);
      break;
    case VT_BOOL:
      pub = nh.advertise<Bool>(name, MESSAGE_QUEUE,
          (ros::SubscriberStatusCallback)Connected);
      bval.data = atoi(vecStr.front().c_str());
      WaitForConnect();
      pub.publish(bval);
      break;
    case (VT_ARRAY | VT_R4):
      pub = nh.advertise<Float32MultiArray>(name, MESSAGE_QUEUE,
          (ros::SubscriberStatusCallback)Connected);
      BOOST_FOREACH(std::string str, vecStr) {
        faryval.data.push_back(atof(str.c_str()));
      }
      WaitForConnect();
      pub.publish(faryval);
      break;
    case (VT_ARRAY | VT_R8):
      pub = nh.advertise<Float64MultiArray>(name, MESSAGE_QUEUE,
          (ros::SubscriberStatusCallback)Connected);
      BOOST_FOREACH(std::string str, vecStr) {
        daryval.data.push_back(atof(str.c_str()));
      }
      WaitForConnect();
      pub.publish(daryval);
      break;
  }
}

void WriteInt32(const std::string& name)
{
  ros::NodeHandle nh;
  ros::Publisher  pub;

  Int32 ival;

  std::cout << "Please input value/number: ";
  std::cin >> ival.data;
  pub = nh.advertise<Int32>(name, MESSAGE_QUEUE,
      (ros::SubscriberStatusCallback)Connected);
  WaitForConnect();
  pub.publish(ival);
}

void WriteFloat32(const std::string& name)
{
  ros::NodeHandle nh;
  ros::Publisher  pub;

  Float32 fltval;

  std::cout << "Please input value/number: ";
  std::cin >> fltval.data;
  pub = nh.advertise<Float32>(name, MESSAGE_QUEUE,
      (ros::SubscriberStatusCallback)Connected);
  WaitForConnect();
  pub.publish(fltval);
}

void Callback_MoveStringFeedback(const MoveStringActionFeedbackConstPtr& msg)
{
  PrintFeedback(msg->feedback.pose);
}

void MoveString(const std::string& name, float timeout)
{
  ros::NodeHandle nh;
  SimpleClientGoalState gs(SimpleClientGoalState::PENDING, "");

  ros::Subscriber sub = nh.subscribe<MoveStringActionFeedback>
    (name + "/feedback", MESSAGE_QUEUE, &Callback_MoveStringFeedback);

  SimpleActionClient<MoveStringAction> acMvStr(name, true);
  MoveStringGoal           mvStrGoal;
  MoveStringResultConstPtr mvStrRes;

  std::cout << "Please input interpolation (PTP:=1, CP:=2): ";
  std::cin >> mvStrGoal.comp;
  std::cout << "Please input pose (ex.: J(0,0,0,0,0,0,0,0)): ";
  ReadLine(mvStrGoal.pose);
  std::cout << "Please input option (If skip, input _): ";
  ReadLine(mvStrGoal.option);
  if(!mvStrGoal.option.compare("_")) mvStrGoal.option = "";

  boost::thread th(&PrintThread);

  acMvStr.sendGoal(mvStrGoal);
  if(!acMvStr.waitForResult(ros::Duration(timeout))) {
    acMvStr.cancelGoal();
  }

  bPrint = true;
  th.join();

  while(ros::ok()) {
    gs = acMvStr.getState();
    if(gs.isDone()) break;
    ros::Duration(1).sleep();
  }

  mvStrRes = acMvStr.getResult();

  ROS_INFO("State: %s, Result: %X", gs.toString().c_str(), mvStrRes->HRESULT);
}

void Callback_MoveValueFeedback(const MoveValueActionFeedbackConstPtr& msg)
{
  PrintFeedback(msg->feedback.pose);
}

void MoveValue(const std::string& name, float timeout)
{
  ros::NodeHandle nh;
  SimpleClientGoalState gs(SimpleClientGoalState::PENDING, "");

  std::string strTmp;
  std::vector<std::string> vecStr;

  ros::Subscriber sub = nh.subscribe<MoveValueActionFeedback>
    (name + "/feedback", MESSAGE_QUEUE, &Callback_MoveValueFeedback);

  SimpleActionClient<MoveValueAction> acMvVal(name, true);
  MoveValueGoal           mvValGoal;
  MoveValueResultConstPtr mvValRes;

  std::cout << "Please input interpolation (PTP:=1, CP:=2): ";
  std::cin >> mvValGoal.comp;
  std::cout << "Please input pose (ex.: 0,0,0,0,0,0,0,0): ";
  ReadLine(strTmp, " ");
  boost::split(vecStr, strTmp, boost::is_any_of(","));
  BOOST_FOREACH(std::string str, vecStr) {
    mvValGoal.pose.value.push_back(atof(str.c_str()));
  }
  std::cout << "Please input type (P: 0, T: 1, J: 2, V: 3): ";
  std::cin >> mvValGoal.pose.type;
  std::cout << "Please input pass (@P: -1, @E: -2, @0: 0, @n: n): ";
  std::cin >> mvValGoal.pose.pass;
  std::cout << "Please input exjoints mode (EX: 1, EXA: 2, No exjoints: 0): ";
  std::cin >> mvValGoal.pose.exjoints.mode;
  if(mvValGoal.pose.exjoints.mode != 0) {
    while(true) {
      std::cout << "Please input exjoint value (ex.: 7,30.5, If skip input _): ";
      ReadLine(strTmp, " ");
      if(!strTmp.compare("_")) break;
      boost::split(vecStr, strTmp, boost::is_any_of(","));
      Joints jnt;
      jnt.joint = atoi(vecStr.at(0).c_str());
      jnt.value = atof(vecStr.at(1).c_str());
      mvValGoal.pose.exjoints.joints.push_back(jnt);
    }
  }
  std::cout << "Please input option (If skip, input _): ";
  ReadLine(mvValGoal.option);
  if(!mvValGoal.option.compare("_")) mvValGoal.option = "";

  boost::thread th(&PrintThread);

  acMvVal.sendGoal(mvValGoal);
  if(!acMvVal.waitForResult(ros::Duration(timeout))) {
    acMvVal.cancelGoal();
  }

  bPrint = true;
  th.join();

  while(ros::ok()) {
    gs = acMvVal.getState();
    if(gs.isDone()) break;
    ros::Duration(1).sleep();
  }

  mvValRes = acMvVal.getResult();

  ROS_INFO("State: %s, Result: %X", gs.toString().c_str(), mvValRes->HRESULT);
}

void Callback_DriveStringFeedback(const DriveStringActionFeedbackConstPtr& msg)
{
  PrintFeedback(msg->feedback.pose);
}

void DriveString(const std::string& name, float timeout)
{
  ros::NodeHandle nh;
  SimpleClientGoalState gs(SimpleClientGoalState::PENDING, "");

  ros::Subscriber sub = nh.subscribe<DriveStringActionFeedback>
    (name + "/feedback", MESSAGE_QUEUE, &Callback_DriveStringFeedback);

  SimpleActionClient<DriveStringAction> acDrvVal(name, true);
  DriveStringGoal           drvStrGoal;
  DriveStringResultConstPtr drvStrRes;

  std::cout << "Please input pose (ex.: @0 (1, 10), (2, 20)): ";
  ReadLine(drvStrGoal.pose);
  std::cout << "Please input option (If skip, input _): ";
  ReadLine(drvStrGoal.option);
  if(!drvStrGoal.option.compare("_")) drvStrGoal.option = "";

  boost::thread th(&PrintThread);

  acDrvVal.sendGoal(drvStrGoal);
  if(!acDrvVal.waitForResult(ros::Duration(timeout))) {
    acDrvVal.cancelGoal();
  }

  bPrint = true;
  th.join();

  while(ros::ok()) {
    gs = acDrvVal.getState();
    if(gs.isDone()) break;
    ros::Duration(1).sleep();
  }

  drvStrRes = acDrvVal.getResult();

  ROS_INFO("State: %s, Result: %X", gs.toString().c_str(), drvStrRes->HRESULT);
}

void Callback_DriveValueFeedback(const DriveValueActionFeedbackConstPtr& msg)
{
  PrintFeedback(msg->feedback.pose);
}

void DriveValue(const std::string& name, float timeout)
{
  ros::NodeHandle nh;
  SimpleClientGoalState gs(SimpleClientGoalState::PENDING, "");

  std::string strTmp;
  std::vector<std::string> vecStr;

  ros::Subscriber sub = nh.subscribe<DriveValueActionFeedback>
    (name + "/feedback", MESSAGE_QUEUE, &Callback_DriveValueFeedback);

  SimpleActionClient<DriveValueAction> acDrvVal(name, true);
  DriveValueGoal           drvValGoal;
  DriveValueResultConstPtr drvValRes;

  std::cout << "Please input pass (@P: -1, @E: -2, @0: 0, @n: n): ";
  std::cin >> drvValGoal.pass;
  while(true) {
    std::cout << "Please input joint value (ex.: 7,30.5, If skip input _): ";
    ReadLine(strTmp, " ");
    if(!strTmp.compare("_")) break;
    boost::split(vecStr, strTmp, boost::is_any_of(","));
    Joints jnt;
    jnt.joint = atoi(vecStr.at(0).c_str());
    jnt.value = atof(vecStr.at(1).c_str());
    drvValGoal.pose.push_back(jnt);
  }
  std::cout << "Please input option (If skip, input _): ";
  ReadLine(drvValGoal.option);
  if(!drvValGoal.option.compare("_")) drvValGoal.option = "";

  boost::thread th(&PrintThread);

  acDrvVal.sendGoal(drvValGoal);
  if(!acDrvVal.waitForResult(ros::Duration(timeout))) {
    acDrvVal.cancelGoal();
  }

  bPrint = true;
  th.join();

  while(ros::ok()) {
    gs = acDrvVal.getState();
    if(gs.isDone()) break;
    ros::Duration(1).sleep();
  }

  drvValRes = acDrvVal.getResult();

  ROS_INFO("State: %s, Result: %X", gs.toString().c_str(), drvValRes->HRESULT);
}

int main(int argc, char** argv)
{
  if(argc < 3) {
    std::cout << "Usage: denso_robot_core mode topic [timeout]" << std::endl;
    std::cout << "mode: 1:= Write variable."                    << std::endl;
    std::cout << "      2:= Write ID."                          << std::endl;
    std::cout << "      3:= Change arm group."                  << std::endl;
    std::cout << "      4:= Move robot with string."            << std::endl;
    std::cout << "      5:= Move robot with value."             << std::endl;
    std::cout << "      6:= Drive robot with string."           << std::endl;
    std::cout << "      7:= Drive robot with value."            << std::endl;
    std::cout << "      8:= Change robot speed."                << std::endl;
    std::cout << "      9:= Change tool or work number."        << std::endl;
    return -1;
  }

  ros::init(argc, argv, "denso_robot_core_test");

  int mode = atoi(argv[1]);

  float timeout = DEFAULT_TIMEOUT;
  if(3 < argc) {
    timeout = atof(argv[3]);
  }

  std::cin.putback('\n');

  switch(mode) {
    case WRITE_VARIABLE:
      WriteVariable(argv[2]);
      break;
    case WRITE_ID:
    case ARM_GROUP:
    case CHANGE_TOOLWORK:
      WriteInt32(argv[2]);
      break;
    case SPEED:
      WriteFloat32(argv[2]);
      break;
    case MOVE_STRING:
      MoveString(argv[2], timeout);
      break;
    case MOVE_VALUE:
      MoveValue(argv[2], timeout);
      break;
    case DRIVE_STRING:
      DriveString(argv[2], timeout);
      break;
    case DRIVE_VALUE:
      DriveValue(argv[2], timeout);
      break;
  }

  return 0;
}
