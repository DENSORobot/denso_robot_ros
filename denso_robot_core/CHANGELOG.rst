^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package denso_robot_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2021-06-02)
------------------
* Add "ResetStoState" to clear the STO state for RC9 and COBOTTA
* Add "ManualResetPreparation" to confirm safey-related command for COBOTTA
* Add "MotionPreparation" to execute Motion preparation for COBOTTA
* Add "AutoCal" to execute CALSET for COBOTTA
* Change "ClearError" to execute "ManualResetPreparation" first for COBOTTA
* Change most of the macros to constants

3.1.2 (2021-04-02)
------------------

3.1.1 (2021-03-03)
------------------

3.1.0 (2020-12-23)
------------------
* Add supporting for RC9
* Add bcap_slave_control_cycle_msec
* Add 4 functions about error
* Upgrade tinyxml2 to official v3.0.0. for Melodic

3.0.4 (2019-11-27)
------------------

3.0.3 (2019-09-23)
------------------
* Change bCap service UDP timeout

3.0.2 (2017-12-15)
------------------
* Add I/O function for denso_robot_core
* Remove new line from ROS_INFO,ROS_WARN,ROS_ERROR
* Change descriptions and add url, author
* Check the catkin_lint result
* Contributors: MIYAKOSHI Yoshihiro

Forthcoming
-----------
* update version to 3.0.0
* first commit
* Contributors: MIYAKOSHI Yoshihiro
