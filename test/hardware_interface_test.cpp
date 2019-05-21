#include "ros_ethercat_driver/hardware_interface/ros_ethercat_hardware_interface.hpp"

int main(int argc, char *argv[])
{
  ros_ethercat_driver::RobotStateEtherCATHardwareInterface EtherCAT_HW;
  EtherCAT_HW.ifname = argv[1];
  EtherCAT_HW.EtherCATInit();
  return 0;
}
