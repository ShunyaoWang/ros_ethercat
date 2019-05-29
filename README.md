# ros_ethercat_driver
<img src = /assets/EtherCAT-16-9.jpg width = 30% height = 30% align = center />

A Ros EtherCAT hardware interface for ros_control based on **SOEM**


## Build Dependencies
- [soem](https://github.com/mgruhler/soem)
  > `sudo apt-get install ros-kinetic-soem`

## Install
- Clone this repository to you catkin workspace and then run
  > `catkin_make`

## Usage
- Fisrt connect the EtherCAT Wire to the Ethernet port of your computer, then, **Disconnect** the wired connection
- There is a hardware_interface for ros_control, which can be loaded in "[balance_controller]()", use roslaunch.
  ```
  sudo su
  source .bashrc
  roslaunch balance_controller balance_controller_manager.launch
  ```
- There a also a serperate **test node** for ANYDrive, configure it to joint velocity mode and run for a few seconds.
  ```
  sudo su
  source .bashrc
  rosrun ros_ethercat_driver anydrive_test eno1
  ```
## rqt GUI for motor test
- Fisrt make sure a **roscore** is running.
- Then enter the su mode and run
  ```
  sudo su
  source .bashrc
  roslaunch rqt_ethercat_test_plugin ethercat_test.launch
  ```
- There is a GUI like this
  ![ethercat_test_gui](/assets/ethercat_test_gui.png)
  - Fisrt, input the Ethernet Port, for example 'eno1'.
  - Then, click button 'connect', if success, there will be a list of device in the Table.

  ## TODO
  Auto Switch between Golden Twitter and ANYdrive.
