#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Example of ChangeScene for COBOTTA PRO
#   Change Scene(0,0) from default Scene.
#   RC9: v1.4.0 or later
#   denso_robot_ros: v3.3.0 or later
#
# Usage:
# $ roslaunch denso_robot_bringup <robot_name>_bringup.launch sim:=false \
#             ip_address:=192.168.0.1 send_format:=0 recv_format:=2
# $ roslaunch bcap_service bcap_service.launch ip_address:=192.168.0.1
# $ rosrun denso_robot_bringup example_change_scene.py _robot_name:=<robot_name>
#
# Reference:
#  - b-CAP_Guide_RC9_en.pdf
#  - RC9_ProvGuide_en.pdf
#  - COBOTTA PRO User Manuals
#
# Copyright (C) 2023  DENSO WAVE INCORPORATED
import sys
import math
import ctypes
from enum import IntEnum

import rosnode
import rospy
import moveit_commander

from std_msgs.msg import Int32
from bcap_service.srv import bcap, bcapRequest
from bcap_service.msg import variant

# Example Target Scene "<Scene Number>,<Sub-Scene Number>"
_EXAMPLE_SCENE_NUMBER = "0,0"
# Example Target Position
_EXAMPLE_POSITION = [0, -45, 135, 0, 45, 0]
# b-CAP slvMove Speed [default:5%]
_SLVMOVE_SPEED = 0.05
# b-CAP Slave Mode
_bcap_slave_mode = 0


# b-CAP function ID
#   cf. b-CAP_Guide_RC9_en.pdf
class BcapFuncId(IntEnum):
    CONTROLLER_CONNECT = 3
    CONTROLLER_DISCONNECT = 4
    CONTROLLER_GETROBOT = 7
    CONTROLLER_EXECUTE = 17
    ROBOT_EXECUTE = 64
    ROBOT_RELEASE = 84
    ROBOT_MOVE = 72


# CaoRobot::Move() interpolation
#   cf. RC9_ProvGuide_en.pdf
class CaoRobotMove(IntEnum):
    MOVE_P = 1
    MOVE_L = 2
    MOVE_C = 3
    MOVE_S = 4


# Variant type of ORiN2/RAC
class VarType(IntEnum):
    VT_I2 = 2
    VT_I4 = 3
    VT_R4 = 4
    VT_R8 = 5
    VT_CY = 6
    VT_DATE = 7
    VT_BSTR = 8
    VT_BOOL = 11
    VT_VARIANT = 12
    VT_U11 = 17
    VT_ARRAY = 0x2000


def usage():
    rospy.logerr("Usage:\n"
                 "  roslaunch denso_robot_bringup <robot_name>_bringup.launch"
                 " sim:=false ip_address:=<IP address> send_format:=0 recv_format:=2\n"
                 "  roslaunch bcap_service bcap_service.launch ip_address:=<IP address>\n"
                 "  rosrun denso_robot_bringup example_change_scene.py"
                 " _robot_name:=<robot_name>")


def convert_error_code(hresult):
    return hex(ctypes.c_uint(hresult).value)


def callback_curmode(message):
    global _bcap_slave_mode
    _bcap_slave_mode = int(message.data)


def change_bcap_slave_mode(pub_changemode, mode):
    if _bcap_slave_mode == mode:
        return

    pub_changemode.publish(mode)
    while (not rospy.is_shutdown()) and (_bcap_slave_mode != mode):
        rospy.sleep(0.5)  # 500ms


# GetErrorDescription
#   b-CAP: Robot_Execute(GetErrorDescription)
#   ORiN2: CaoController::Execute(GetErrorDescription)
def print_error_description(bcap_service, rc9, hresult):
    bcap_request = bcapRequest()
    bcap_request.func_id = BcapFuncId.CONTROLLER_EXECUTE
    bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9),
                            variant(vt=VarType.VT_BSTR, value='GetErrorDescription'),
                            variant(vt=VarType.VT_I4, value=str(hresult))]
    res = bcap_service(bcap_request)
    if res.HRESULT < 0:
        return

    rospy.logerr("%s: %s", convert_error_code(hresult), res.vntRet.value)


def get_current_scene(bcap_service, rc9, rc9robot):
    # CurScene: Get current Scene Number. Number 11 is default Scene.
    #   b-CAP: Robot_Execute(CurScene)
    #   ORiN2: CaoRobot::Execute(CurScene)
    bcap_request = bcapRequest()
    bcap_request.func_id = BcapFuncId.ROBOT_EXECUTE
    bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9robot),
                            variant(vt=VarType.VT_BSTR, value='CurScene'),
                            variant(vt=VarType.VT_BSTR, value='')]
    res = bcap_service(bcap_request)
    if res.HRESULT < 0:
        print_error_description(bcap_service, rc9, res.HRESULT)
        raise Exception("bcap_service: RC9Robot: Failed to execute CurScene. error="
                        + convert_error_code(res.HRESULT))
    scene_number = res.vntRet.value

    # CurSubScene: Get current Sub-Scene Number
    #   b-CAP: Robot_Execute(CurSubScene)
    #   ORiN2: CaoRobot::Execute(CurSubScene)
    bcap_request = bcapRequest()
    bcap_request.func_id = BcapFuncId.ROBOT_EXECUTE
    bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9robot),
                            variant(vt=VarType.VT_BSTR, value='CurSubScene'),
                            variant(vt=VarType.VT_BSTR, value='')]
    res = bcap_service(bcap_request)
    if res.HRESULT < 0:
        print_error_description(bcap_service, rc9, res.HRESULT)
        raise Exception("bcap_service: RC9Robot: Failed to execute CurSubScene. error="
                        + convert_error_code(res.HRESULT))
    sub_scene_number = res.vntRet.value

    return scene_number, sub_scene_number


def main():
    global _bcap_slave_mode
    _bcap_slave_mode = 0
    rc9 = 0
    rc9robot = 0
    take_arm = False

    rospy.init_node('example_change_scene')
    rospy.loginfo("Start Example of ChangeScene...")

    robot_name = rospy.get_param("~robot_name", "")
    if not robot_name or len(robot_name) <= 0:
        rospy.logerr("Invalid parameter: robot_name")
        usage()
        sys.exit(1)
    rospy.loginfo("robot_name: %s", robot_name)

    rospy.loginfo("Check if /%s/denso_robot_control is up...", robot_name)
    node_list = rosnode.get_node_names(namespace=robot_name)
    if not '/' + robot_name + '/denso_robot_control' in node_list:
        rospy.logerr("No such node: /%s/denso_robot_control", robot_name)
        usage()
        sys.exit(1)

    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander('arm')
    pub_changemode = rospy.Publisher('/' + robot_name + '/ChangeMode',
                                     Int32, queue_size=1)
    sub_curmode = rospy.Subscriber('/' + robot_name + '/CurMode',
                                   Int32, callback_curmode)
    while (pub_changemode.get_num_connections() < 1)\
            and (sub_curmode.get_num_connections() < 1):
        rospy.sleep(0.5)

    rospy.loginfo("Waiting for /bcap_service...")
    rospy.wait_for_service('/bcap_service')
    try:
        rospy.loginfo("Connecting to /bcap_service...")
        bcap_service = rospy.ServiceProxy('/bcap_service', bcap)

        # Connect to RC9 controller using b-CAP Service
        #  - b-CAP: Controller_Connect()
        #  - ORiN2: CaoWorkspace::AddController()
        rospy.loginfo("bcap_service: Controller_Connect()...")
        bcap_request = bcapRequest()
        bcap_request.func_id = BcapFuncId.CONTROLLER_CONNECT
        bcap_request.vntArgs = [variant(vt=VarType.VT_BSTR, value='example_controller_name'),
                                variant(vt=VarType.VT_BSTR, value='CaoProv.DENSO.VRC9'),
                                variant(vt=VarType.VT_BSTR, value='localhost'),
                                variant(vt=VarType.VT_BSTR, value='@IfNotMember')]
        res = bcap_service(bcap_request)
        if res.HRESULT < 0:
            raise Exception("bcap_service: Failed to connect to RC9 controller. error="
                            + convert_error_code(res.HRESULT))
        rc9 = res.vntRet.value
        rospy.loginfo("bcap_service: Connected to RC9. Controller handle is %s", rc9)

        # Get a robot Handle from controller
        #  - b-CAP: Controller_GetRobot()
        #  - ORiN2: CaoController::AddRobot()
        rospy.loginfo("bcap_service: RC9: Controller_GetRobot()...")
        bcap_request = bcapRequest()
        bcap_request.func_id = BcapFuncId.CONTROLLER_GETROBOT
        bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9),
                                variant(vt=VarType.VT_BSTR, value='example_robot_name'),
                                variant(vt=VarType.VT_BSTR, value='@IfNotMember')]
        res = bcap_service(bcap_request)
        if res.HRESULT < 0:
            raise Exception("bcap_service: RC9: Failed to get the robot handle. error="
                            + convert_error_code(res.HRESULT))
        rc9robot = res.vntRet.value
        rospy.loginfo("bcap_service: RC9: Robot handle is %s", rc9robot)

        # slvGetMode: Acquire the current setting of the Slave Mode
        #   - b-CAP: Robot_Execute(slvGetMode)
        #   - ORiN2: CaoRobot::Execute(slvGetMode)
        rospy.loginfo("bcap_service: RC9Robot: Executing slvGetMode...")
        bcap_request = bcapRequest()
        bcap_request.func_id = BcapFuncId.ROBOT_EXECUTE
        bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9robot),
                                variant(vt=VarType.VT_BSTR, value='slvGetMode'),
                                variant(vt=VarType.VT_BSTR, value='')]
        res = bcap_service(bcap_request)
        if res.HRESULT < 0:
            print_error_description(bcap_service, rc9, res.HRESULT)
            raise Exception("bcap_service: RC9Robot: Failed to execute slvGetMode. error="
                            + convert_error_code(res.HRESULT))
        _bcap_slave_mode = int(res.vntRet.value)
        rospy.loginfo("bcap_service: RC9Robot: Current b-CAP Slave Mode is 0x%x",
                      _bcap_slave_mode)

        # Stop b-CAP Slave Mode
        rospy.loginfo("Stopping b-CAP Slave Mode...")
        change_bcap_slave_mode(pub_changemode, 0)

        # ClearError: Clear an error that occurs in the controller
        #   b-CAP: Controller_Execute(ClearError)
        #   ORiN2: CaoController::Execute(ClearError)
        rospy.loginfo("bcap_service: RC9: Executing ClearError...")
        bcap_request = bcapRequest()
        bcap_request.func_id = BcapFuncId.CONTROLLER_EXECUTE
        bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9),
                                variant(vt=VarType.VT_BSTR, value='ClearError'),
                                variant(vt=VarType.VT_BSTR, value='')]
        res = bcap_service(bcap_request)
        if res.HRESULT < 0:
            print_error_description(bcap_service, rc9, res.HRESULT)
            raise Exception("bcap_service: RC9: Failed to execute ClearError. error="
                            + convert_error_code(res.HRESULT))

        # ManualReset: Clear safety-state
        #  b-CAP: Controller_Execute(ManualReset)
        #  ORiN2: CaoController::Execute(ManualReset)
        rospy.loginfo("bcap_service: RC9: Executing ManualReset...")
        bcap_request = bcapRequest()
        bcap_request.func_id = BcapFuncId.CONTROLLER_EXECUTE
        bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9),
                                variant(vt=VarType.VT_BSTR, value='ManualReset'),
                                variant(vt=VarType.VT_BSTR, value='')]
        res = bcap_service(bcap_request)
        if res.HRESULT < 0:
            print_error_description(bcap_service, rc9, res.HRESULT)
            raise Exception("bcap_service: RC9: Failed to execute ManualReset. error="
                            + convert_error_code(res.HRESULT))

        # Motor: Turn on the motor
        #  b-CAP: Robot_Execute(Motor)
        #  ORiN2: CaoRobot::Execute(Motor)
        rospy.loginfo("bcap_service: RC9Robot: Running Motor...")
        bcap_request = bcapRequest()
        bcap_request.func_id = BcapFuncId.ROBOT_EXECUTE
        bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9robot),
                                variant(vt=VarType.VT_BSTR, value='Motor'),
                                variant(vt=VarType.VT_I4, value='1')]
        res = bcap_service(bcap_request)
        if res.HRESULT < 0:
            print_error_description(bcap_service, rc9, res.HRESULT)
            raise Exception("bcap_service: RC9Robot: Failed to run motor. error="
                            + convert_error_code(res.HRESULT))

        # TakeArm: Request to get control authority
        #   b-CAP: Robot_Execute(TakeArm)
        #   ORiN2: CaoRobot::Execute(TakeArm)
        rospy.loginfo("bcap_service: RC9Robot: Executing TakeArm...")
        bcap_request = bcapRequest()
        bcap_request.func_id = BcapFuncId.ROBOT_EXECUTE
        bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9robot),
                                variant(vt=VarType.VT_BSTR, value='TakeArm'),
                                variant(vt=(VarType.VT_ARRAY | VarType.VT_I4),
                                        value='0,1')]
        res = bcap_service(bcap_request)
        if res.HRESULT < 0:
            print_error_description(bcap_service, rc9, res.HRESULT)
            raise Exception("bcap_service: RC9Robot: Failed to execute TakeArm. error="
                            + convert_error_code(res.HRESULT))
        take_arm = True

        # CurScene/SurSubScene: Get current Scene Number and Sub-Scene Number
        rospy.loginfo("bcap_service: RC9Robot: Executing CurScene and CurSubScene...")
        scene_number, sub_scene_number = get_current_scene(bcap_service, rc9, rc9robot)
        rospy.loginfo("bcap_service: RC9Robot: Current Scene is (%s,%s)",
                      scene_number, sub_scene_number)

        # Move SafetyP0: Required before ChangeScene
        #   b-CAP: Robot_Move(MOVE P)
        #   ORiN2: CaoRobot::Move(MOVE P)
        #   If Tool Number and Work Number are specified in Scene Parameter Setting,
        #   change them with CaoRobot::Change().
        #   Select a motion option of Move command suitable for Scene Parameter Setting.
        rospy.loginfo("bcap_service: RC9Robot: Moving to SafetyP0 position before ChangeScene...")
        bcap_request = bcapRequest()
        bcap_request.func_id = BcapFuncId.ROBOT_MOVE
        bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9robot),
                                variant(vt=VarType.VT_I4, value=str(CaoRobotMove.MOVE_P.value)),
                                variant(vt=VarType.VT_BSTR, value='SafetyP0'),
                                variant(vt=VarType.VT_BSTR, value='')]
        res = bcap_service(bcap_request)
        if res.HRESULT < 0:
            print_error_description(bcap_service, rc9, res.HRESULT)
            raise Exception("bcap_service: RC9Robot: Failed to move SafetyP0. error="
                            + convert_error_code(res.HRESULT))

        # ChangeScene: Change current Scene
        #   b-CAP: Robot_Execute(ChangeScene)
        #   ORiN2: CaoRobot::Execute(ChangeScene)
        rospy.loginfo("bcap_service: RC9Robot: Executing ChangeScene(%s)...",
                      _EXAMPLE_SCENE_NUMBER)
        bcap_request = bcapRequest()
        bcap_request.func_id = BcapFuncId.ROBOT_EXECUTE
        bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9robot),
                                variant(vt=VarType.VT_BSTR, value='ChangeScene'),
                                variant(vt=(VarType.VT_ARRAY | VarType.VT_I4),
                                        value=_EXAMPLE_SCENE_NUMBER)]
        res = bcap_service(bcap_request)
        if res.HRESULT < 0:
            print_error_description(bcap_service, rc9, res.HRESULT)
            raise Exception("bcap_service: RC9Robot: Failed to execute ChangeScene. error="
                            + convert_error_code(res.HRESULT))

        # GiveArm: Request to release control authority
        #   b-CAP: Robot_Execute(GiveArm)
        #   ORiN2: CaoRobot::Execute(GiveArm)
        rospy.loginfo("bcap_service: RC9Robot: Executing GiveArm...")
        bcap_request = bcapRequest()
        bcap_request.func_id = BcapFuncId.ROBOT_EXECUTE
        bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9robot),
                                variant(vt=VarType.VT_BSTR, value='GiveArm'),
                                variant(vt=VarType.VT_BSTR, value='')]
        res = bcap_service(bcap_request)
        if res.HRESULT < 0:
            print_error_description(bcap_service, rc9, res.HRESULT)
            raise Exception("bcap_service: RC9Robot: Failed to execute GiveArm. error=%x"
                            + convert_error_code(res.HRESULT))
        take_arm = False

        # Start b-Cap Slave
        rospy.loginfo("Starting b-CAP Slave Mode...")
        change_bcap_slave_mode(pub_changemode, 0x202)

        # Robot moves by MoveIt!
        rospy.loginfo("Move(slvMove) to " + str(_EXAMPLE_POSITION) + "...")
        move_group.set_max_acceleration_scaling_factor(_SLVMOVE_SPEED)
        move_group.set_max_velocity_scaling_factor(_SLVMOVE_SPEED)
        pose = [x / 180.0 * math.pi for x in _EXAMPLE_POSITION]
        move_group.go(pose, wait=True)
        move_group.stop()

    except rospy.ServiceException as se:
        rospy.logerr("rospy.ServiceException: %s", se)

    except Exception as e:
        rospy.logerr("%s: %s", e.__class__.__name__, e.message)

    finally:
        rospy.loginfo("Cleaning up...")
        # GiveArm
        if rc9robot > 0 and take_arm:
            rospy.loginfo("bcap_service: RC9Robot: Executing GiveArm...")
            bcap_request = bcapRequest()
            bcap_request.func_id = BcapFuncId.ROBOT_EXECUTE
            bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9robot),
                                    variant(vt=VarType.VT_BSTR, value='GiveArm'),
                                    variant(vt=VarType.VT_BSTR, value='')]
            bcap_service(bcap_request)

        if rc9robot > 0:
            # Release robot handle
            #   b-CAP: Robot_Release()
            #   ORiN2: CaoRobots::Remove()
            rospy.loginfo("bcap_service: RC9Robot: Release robot handle...")
            bcap_request = bcapRequest()
            bcap_request.func_id = BcapFuncId.ROBOT_RELEASE
            bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9robot)]
            bcap_service(bcap_request)

        if rc9 > 0:
            # Disconnect from the controller
            #   b-CAP: Controller_Disconnect()
            #   ORiN2: CaoWorkspace::Controllers::Remove()
            rospy.loginfo("bcap_service: RC9: Disconnect from the controller...")
            bcap_request = bcapRequest()
            bcap_request.func_id = BcapFuncId.CONTROLLER_DISCONNECT
            bcap_request.vntArgs = [variant(vt=VarType.VT_I4, value=rc9)]
            bcap_service(bcap_request)
            rospy.loginfo("bcap_service: RC9: Disconnected.")


if __name__ == "__main__":
    main()
