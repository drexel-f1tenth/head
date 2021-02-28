#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathfinder");
  ros::NodeHandle node;

  auto scan_cb = [](sensor_msgs::LaserScan const& frame) {
    std::cout << frame << std::endl;
  };

  using ScanCB = void(*)(sensor_msgs::LaserScan const&);
  auto sub = node.subscribe("/scan", 1, (ScanCB)scan_cb);

  ros::spin();
  return 0;
}
