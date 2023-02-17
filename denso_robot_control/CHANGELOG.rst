^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package denso_robot_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.3.0 (2023-02-14)
------------------
* Add supporting for COBOTTA PRO
* Change "ResetStoState" to "ManualReset" in the initialization process
* Modify to use reset_controllers when b-CAP Slave is enabled
* Add parameter to specify bcap_slave_mode at startup

3.2.0 (2021-06-02)
------------------
* Add "ResetStoState" to initialization

3.1.2 (2021-04-02)
------------------

3.1.1 (2021-03-03)
------------------
* Add ros::spinOnce() for the callbacks

3.1.0 (2020-12-23)
------------------
* Add supporting for RC9
* Add bcap_slave_control_cycle_msec
* Add error descriptions of controller
* Fix send_format/recv_format
* Fix Int32 to UInt32 of MiniIO and HandIO
* Remove first 5-seconds-sleep in main()

3.0.4 (2019-11-27)
------------------

3.0.3 (2019-09-23)
------------------
* Change to sleep in only slave sync mode

3.0.2 (2017-12-15)
------------------
* Add clear error step before motor on
* Add subscriber and publisher for I/O function
* Add I/O function for denso_robot_control
* Remove new line from ROS_INFO,ROS_WARN,ROS_ERROR
* Change descriptions and add url, author
* Check the catkin_lint result
* Contributors: MIYAKOSHI Yoshihiro

Forthcoming
-----------
* update version to 3.0.0
* first commit
* Contributors: MIYAKOSHI Yoshihiro
