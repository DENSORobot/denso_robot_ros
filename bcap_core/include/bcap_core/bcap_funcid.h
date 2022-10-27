#ifndef BCAP_FUNCID_H_
#define BCAP_FUNCID_H_

/**
 * @file    bcap_funcid.h
 * @brief   b-CAP Function ID file.
 * @details Defines b-CAP functions IDs.
 *
 * @version 1.0
 * @date    2014/12/24
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

#define ID_SERVICE_START (1)
#define ID_SERVICE_STOP  (2)

#define ID_CONTROLLER_CONNECT           ( 3)
#define ID_CONTROLLER_DISCONNECT        ( 4)
#define ID_CONTROLLER_GETEXTENSION      ( 5)
#define ID_CONTROLLER_GETFILE           ( 6)
#define ID_CONTROLLER_GETROBOT          ( 7)
#define ID_CONTROLLER_GETTASK           ( 8)
#define ID_CONTROLLER_GETVARIABLE       ( 9)
#define ID_CONTROLLER_GETCOMMAND        (10)
#define ID_CONTROLLER_GETEXTENSIONNAMES (11)
#define ID_CONTROLLER_GETFILENAMES      (12)
#define ID_CONTROLLER_GETROBOTNAMES     (13)
#define ID_CONTROLLER_GETTASKNAMES      (14)
#define ID_CONTROLLER_GETVARIABLENAMES  (15)
#define ID_CONTROLLER_GETCOMMANDNAMES   (16)
#define ID_CONTROLLER_EXECUTE           (17)
#define ID_CONTROLLER_GETMESSAGE        (18)
#define ID_CONTROLLER_GETATTRIBUTE      (19)
#define ID_CONTROLLER_GETHELP           (20)
#define ID_CONTROLLER_GETNAME           (21)
#define ID_CONTROLLER_GETTAG            (22)
#define ID_CONTROLLER_PUTTAG            (23)
#define ID_CONTROLLER_GETID             (24)
#define ID_CONTROLLER_PUTID             (25)

#define ID_EXTENSION_GETVARIABLE      (26)
#define ID_EXTENSION_GETVARIABLENAMES (27)
#define ID_EXTENSION_EXECUTE          (28)
#define ID_EXTENSION_GETATTRIBUTE     (29)
#define ID_EXTENSION_GETHELP          (30)
#define ID_EXTENSION_GETNAME          (31)
#define ID_EXTENSION_GETTAG           (32)
#define ID_EXTENSION_PUTTAG           (33)
#define ID_EXTENSION_GETID            (34)
#define ID_EXTENSION_PUTID            (35)
#define ID_EXTENSION_RELEASE          (36)

#define ID_FILE_GETFILE              (37)
#define ID_FILE_GETVARIABLE          (38)
#define ID_FILE_GETFILENAMES         (39)
#define ID_FILE_GETVARIABLENAMES     (40)
#define ID_FILE_EXECUTE              (41)
#define ID_FILE_COPY                 (42)
#define ID_FILE_DELETE               (43)
#define ID_FILE_MOVE                 (44)
#define ID_FILE_RUN                  (45)
#define ID_FILE_GETDATECREATED       (46)
#define ID_FILE_GETDATELASTACCESSED  (47)
#define ID_FILE_GETDATELASTMODIFIED  (48)
#define ID_FILE_GETPATH              (49)
#define ID_FILE_GETSIZE              (50)
#define ID_FILE_GETTYPE              (51)
#define ID_FILE_GETVALUE             (52)
#define ID_FILE_PUTVALUE             (53)
#define ID_FILE_GETATTRIBUTE         (54)
#define ID_FILE_GETHELP              (55)
#define ID_FILE_GETNAME              (56)
#define ID_FILE_GETTAG               (57)
#define ID_FILE_PUTTAG               (58)
#define ID_FILE_GETID                (59)
#define ID_FILE_PUTID                (60)
#define ID_FILE_RELEASE              (61)

#define ID_ROBOT_GETVARIABLE      (62)
#define ID_ROBOT_GETVARIABLENAMES (63)
#define ID_ROBOT_EXECUTE          (64)
#define ID_ROBOT_ACCELERATE       (65)
#define ID_ROBOT_CHANGE           (66)
#define ID_ROBOT_CHUCK            (67)
#define ID_ROBOT_DRIVE            (68)
#define ID_ROBOT_GOHOME           (69)
#define ID_ROBOT_HALT             (70)
#define ID_ROBOT_HOLD             (71)
#define ID_ROBOT_MOVE             (72)
#define ID_ROBOT_ROTATE           (73)
#define ID_ROBOT_SPEED            (74)
#define ID_ROBOT_UNCHUCK          (75)
#define ID_ROBOT_UNHOLD           (76)
#define ID_ROBOT_GETATTRIBUTE     (77)
#define ID_ROBOT_GETHELP          (78)
#define ID_ROBOT_GETNAME          (79)
#define ID_ROBOT_GETTAG           (80)
#define ID_ROBOT_PUTTAG           (81)
#define ID_ROBOT_GETID            (82)
#define ID_ROBOT_PUTID            (83)
#define ID_ROBOT_RELEASE          (84)

#define ID_TASK_GETVARIABLE      (85)
#define ID_TASK_GETVARIABLENAMES (86)
#define ID_TASK_EXECUTE          (87)
#define ID_TASK_START            (88)
#define ID_TASK_STOP             (89)
#define ID_TASK_DELETE           (90)
#define ID_TASK_GETFILENAME      (91)
#define ID_TASK_GETATTRIBUTE     (92)
#define ID_TASK_GETHELP          (93)
#define ID_TASK_GETNAME          (94)
#define ID_TASK_GETTAG           (95)
#define ID_TASK_PUTTAG           (96)
#define ID_TASK_GETID            (97)
#define ID_TASK_PUTID            (98)
#define ID_TASK_RELEASE          (99)

#define ID_VARIABLE_GETDATETIME    (100)
#define ID_VARIABLE_GETVALUE       (101)
#define ID_VARIABLE_PUTVALUE       (102)
#define ID_VARIABLE_GETATTRIBUTE   (103)
#define ID_VARIABLE_GETHELP        (104)
#define ID_VARIABLE_GETNAME        (105)
#define ID_VARIABLE_GETTAG         (106)
#define ID_VARIABLE_PUTTAG         (107)
#define ID_VARIABLE_GETID          (108)
#define ID_VARIABLE_PUTID          (109)
#define ID_VARIABLE_GETMICROSECOND (110)
#define ID_VARIABLE_RELEASE        (111)

#define ID_COMMAND_EXECUTE       (112)
#define ID_COMMAND_CANCEL        (113)
#define ID_COMMAND_GETTIMEOUT    (114)
#define ID_COMMAND_PUTTIMEOUT    (115)
#define ID_COMMAND_GETSTATE      (116)
#define ID_COMMAND_GETPARAMETERS (117)
#define ID_COMMAND_PUTPARAMETERS (118)
#define ID_COMMAND_GETRESULT     (119)
#define ID_COMMAND_GETATTRIBUTE  (120)
#define ID_COMMAND_GETHELP       (121)
#define ID_COMMAND_GETNAME       (122)
#define ID_COMMAND_GETTAG        (123)
#define ID_COMMAND_PUTTAG        (124)
#define ID_COMMAND_GETID         (125)
#define ID_COMMAND_PUTID         (126)
#define ID_COMMAND_RELEASE       (127)

#define ID_MESSAGE_REPLY           (128)
#define ID_MESSAGE_CLEAR           (129)
#define ID_MESSAGE_GETDATETIME     (130)
#define ID_MESSAGE_GETDESCRIPTION  (131)
#define ID_MESSAGE_GETDESTINATION  (132)
#define ID_MESSAGE_GETNUMBER       (133)
#define ID_MESSAGE_GETSERIALNUMBER (134)
#define ID_MESSAGE_GETSOURCE       (135)
#define ID_MESSAGE_GETVALUE        (136)
#define ID_MESSAGE_RELEASE         (137)

#endif /* BCAP_FUNCID_H_ */
