# ros_ethercat_driver
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

  ## TODO
  Fixed EtherCAT update loop with accurate real_time in hardware_interface
