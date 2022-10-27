#ifndef BCAP_MAPID_H_
#define BCAP_MAPID_H_

/**
 * @file    bcap_mapid.h
 * @brief   b-CAP MAP ID file.
 * @details Defines b-CAP MAP IDs.
 *
 * @version 1.0
 * @date    2015/12/16
 * @author  DENSO WAVE
 *
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

#include "../bcap_funcid.h"

/**
 * @struct MAP_ID
 * @brief  A map for function id information.
 */
struct MAP_ID
{
  int32_t relation_id; /**< Function ID */
  int return_flag;     /**< If 1 then sends the response packet with a argument, else without any argument */
};

static const struct MAP_ID m_map_id[] =
  {
    { 0, 0 },
    { ID_SERVICE_STOP, 0 }, // ID_SERVICE_START
    { -1, 0 }, // ID_SERVICE_STOP
    { ID_CONTROLLER_DISCONNECT, 1 }, // ID_CONTROLLER_CONNECT
    { -1, 0 }, // ID_CONTROLLER_DISCONNECT
    { ID_EXTENSION_RELEASE, 1 }, // ID_CONTROLLER_GETEXTENSION
    { ID_FILE_RELEASE, 1 }, // ID_CONTROLLER_GETFILE
    { ID_ROBOT_RELEASE, 1 }, // ID_CONTROLLER_GETROBOT
    { ID_TASK_RELEASE, 1 }, // ID_CONTROLLER_GETTASK
    { ID_VARIABLE_RELEASE, 1 }, // ID_CONTROLLER_GETVARIABLE
    { ID_COMMAND_RELEASE, 1 }, // ID_CONTROLLER_GETCOMMAND
    { 0, 1 }, // ID_CONTROLLER_GETEXTENSIONNAMES
    { 0, 1 }, // ID_CONTROLLER_GETFILENAMES
    { 0, 1 }, // ID_CONTROLLER_GETROBOTNAMES
    { 0, 1 }, // ID_CONTROLLER_GETTASKNAMES
    { 0, 1 }, // ID_CONTROLLER_GETVARIABLENAMES
    { 0, 1 }, // ID_CONTROLLER_GETCOMMANDNAMES
    { 0, 1 }, // ID_CONTROLLER_EXECUTE
    { ID_MESSAGE_RELEASE, 1 }, // ID_CONTROLLER_GETMESSAGE
    { 0, 1 }, // ID_CONTROLLER_GETATTRIBUTE
    { 0, 1 }, // ID_CONTROLLER_GETHELP
    { 0, 1 }, // ID_CONTROLLER_GETNAME
    { 0, 1 }, // ID_CONTROLLER_GETTAG
    { 0, 0 }, // ID_CONTROLLER_PUTTAG
    { 0, 1 }, // ID_CONTROLLER_GETID
    { 0, 0 }, // ID_CONTROLLER_PUTID
    { ID_VARIABLE_RELEASE, 1 }, // ID_EXTENSION_GETVARIABLE
    { 0, 1 }, // ID_EXTENSION_GETVARIABLENAMES
    { 0, 1 }, // ID_EXTENSION_EXECUTE
    { 0, 1 }, // ID_EXTENSION_GETATTRIBUTE
    { 0, 1 }, // ID_EXTENSION_GETHELP
    { 0, 1 }, // ID_EXTENSION_GETNAME
    { 0, 1 }, // ID_EXTENSION_GETTAG
    { 0, 0 }, // ID_EXTENSION_PUTTAG
    { 0, 1 }, // ID_EXTENSION_GETID
    { 0, 0 }, // ID_EXTENSION_PUTID
    { -1, 0 }, // ID_EXTENSION_RELEASE
    { ID_FILE_RELEASE, 1 }, // ID_FILE_GETFILE
    { ID_VARIABLE_RELEASE, 1 }, // ID_FILE_GETVARIABLE
    { 0, 1 }, // ID_FILE_GETFILENAMES
    { 0, 1 }, // ID_FILE_GETVARIABLENAMES
    { 0, 1 }, // ID_FILE_EXECUTE
    { 0, 0 }, // ID_FILE_COPY
    { 0, 0 }, // ID_FILE_DELETE
    { 0, 0 }, // ID_FILE_MOVE
    { 0, 1 }, // ID_FILE_RUN
    { 0, 1 }, // ID_FILE_GETDATECREATED
    { 0, 1 }, // ID_FILE_GETDATELASTACCESSED
    { 0, 1 }, // ID_FILE_GETDATELASTMODIFIED
    { 0, 1 }, // ID_FILE_GETPATH
    { 0, 1 }, // ID_FILE_GETSIZE
    { 0, 1 }, // ID_FILE_GETTYPE
    { 0, 1 }, // ID_FILE_GETVALUE
    { 0, 0 }, // ID_FILE_PUTVALUE
    { 0, 1 }, // ID_FILE_GETATTRIBUTE
    { 0, 1 }, // ID_FILE_GETHELP
    { 0, 1 }, // ID_FILE_GETNAME
    { 0, 1 }, // ID_FILE_GETTAG
    { 0, 0 }, // ID_FILE_PUTTAG
    { 0, 1 }, // ID_FILE_GETID
    { 0, 0 }, // ID_FILE_PUTID
    { -1, 0 }, // ID_FILE_RELEASE
    { ID_VARIABLE_RELEASE, 1 }, // ID_ROBOT_GETVARIABLE
    { 0, 1 }, // ID_ROBOT_GETVARIABLENAMES
    { 0, 1 }, // ID_ROBOT_EXECUTE
    { 0, 0 }, // ID_ROBOT_ACCELERATE
    { 0, 0 }, // ID_ROBOT_CHANGE
    { 0, 0 }, // ID_ROBOT_CHUCK
    { 0, 0 }, // ID_ROBOT_DRIVE
    { 0, 0 }, // ID_ROBOT_GOHOME
    { 0, 0 }, // ID_ROBOT_HALT
    { 0, 0 }, // ID_ROBOT_HOLD
    { 0, 0 }, // ID_ROBOT_MOVE
    { 0, 0 }, // ID_ROBOT_ROTATE
    { 0, 0 }, // ID_ROBOT_SPEED
    { 0, 0 }, // ID_ROBOT_UNCHUCK
    { 0, 0 }, // ID_ROBOT_UNHOLD
    { 0, 1 }, // ID_ROBOT_GETATTRIBUTE
    { 0, 1 }, // ID_ROBOT_GETHELP
    { 0, 1 }, // ID_ROBOT_GETNAME
    { 0, 1 }, // ID_ROBOT_GETTAG
    { 0, 0 }, // ID_ROBOT_PUTTAG
    { 0, 1 }, // ID_ROBOT_GETID
    { 0, 0 }, // ID_ROBOT_PUTID
    { -1, 0 }, // ID_ROBOT_RELEASE
    { ID_VARIABLE_RELEASE, 1 }, // ID_TASK_GETVARIABLE
    { 0, 1 }, // ID_TASK_GETVARIABLENAMES
    { 0, 1 }, // ID_TASK_EXECUTE
    { 0, 0 }, // ID_TASK_START
    { 0, 0 }, // ID_TASK_STOP
    { 0, 0 }, // ID_TASK_DELETE
    { 0, 1 }, // ID_TASK_GETFILENAME
    { 0, 1 }, // ID_TASK_GETATTRIBUTE
    { 0, 1 }, // ID_TASK_GETHELP
    { 0, 1 }, // ID_TASK_GETNAME
    { 0, 1 }, // ID_TASK_GETTAG
    { 0, 0 }, // ID_TASK_PUTTAG
    { 0, 1 }, // ID_TASK_GETID
    { 0, 0 }, // ID_TASK_PUTID
    { -1, 0 }, // ID_TASK_RELEASE
    { 0, 1 }, // ID_VARIABLE_GETDATETIME
    { 0, 1 }, // ID_VARIABLE_GETVALUE
    { 0, 0 }, // ID_VARIABLE_PUTVALUE
    { 0, 1 }, // ID_VARIABLE_GETATTRIBUTE
    { 0, 1 }, // ID_VARIABLE_GETHELP
    { 0, 1 }, // ID_VARIABLE_GETNAME
    { 0, 1 }, // ID_VARIABLE_GETTAG
    { 0, 0 }, // ID_VARIABLE_PUTTAG
    { 0, 1 }, // ID_VARIABLE_GETID
    { 0, 0 }, // ID_VARIABLE_PUTID
    { 0, 1 }, // ID_VARIABLE_GETMICROSECOND
    { -1, 0 }, // ID_VARIABLE_RELEASE
    { 0, 0 }, // ID_COMMAND_EXECUTE
    { 0, 0 }, // ID_COMMAND_CANCEL
    { 0, 1 }, // ID_COMMAND_GETTIMEOUT
    { 0, 0 }, // ID_COMMAND_PUTTIMEOUT
    { 0, 1 }, // ID_COMMAND_GETSTATE
    { 0, 1 }, // ID_COMMAND_GETPARAMETERS
    { 0, 0 }, // ID_COMMAND_PUTPARAMETERS
    { 0, 1 }, // ID_COMMAND_GETRESULT
    { 0, 1 }, // ID_COMMAND_GETATTRIBUTE
    { 0, 1 }, // ID_COMMAND_GETHELP
    { 0, 1 }, // ID_COMMAND_GETNAME
    { 0, 1 }, // ID_COMMAND_GETTAG
    { 0, 1 }, // ID_COMMAND_PUTTAG
    { 0, 1 }, // ID_COMMAND_GETID
    { 0, 0 }, // ID_COMMAND_PUTID
    { -1, 0 }, // ID_COMMAND_RELEASE
    { 0, 0 }, // ID_MESSAGE_REPLY
    { 0, 0 }, // ID_MESSAGE_CLEAR
    { 0, 1 }, // ID_MESSAGE_GETDATETIME
    { 0, 1 }, // ID_MESSAGE_GETDESCRIPTION
    { 0, 1 }, // ID_MESSAGE_GETDESTINATION
    { 0, 1 }, // ID_MESSAGE_GETNUMBER
    { 0, 1 }, // ID_MESSAGE_GETSERIALNUMBER
    { 0, 1 }, // ID_MESSAGE_GETSOURCE
    { 0, 1 }, // ID_MESSAGE_GETVALUE
    { -1, 0 }, // ID_MESSAGE_RELEASE
  };

#endif /* BCAP_MAPID_H_ */
