/**\file sensor_emulator.cpp
 * \brief ROS Action server to publish sensor_msgs::PointCloud2 from .ply files
 *
 * @version 1.0
 * @author Joana Silva Dias
 */

#include <sensor_emulator_server.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_emulator");

  // action
  SensorEmulatorAction sensor(ros::this_node::getName());

  ros::spin();

  return 0;
}
