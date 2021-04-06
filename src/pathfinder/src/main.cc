#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>
#include <vector>

class Pathfinder
{
  std::vector<float> _angles;

public:
  Pathfinder() {}

  uint8_t update_frame(sensor_msgs::LaserScan const& frame)
  {
    if ((_angles.size() == 0) || (_angles[0] != frame.angle_min))
    {
      _angles.push_back(frame.angle_min);
      for (size_t i = 1; i < frame.ranges.size(); i++)
        _angles.push_back(_angles.back() + frame.angle_increment);
    }
    assert(_angles.size() == frame.ranges.size());

    float min_angle = 0.0;
    float min_range = frame.range_max;
    for (size_t i = 0; i < frame.ranges.size(); i++)
    {
      if (fabs(_angles[i]) > 0.2)
        continue;

      if (frame.ranges[i] < 1.0)
      {
        printf("stop: %.2f, %.2f\n", _angles[i], frame.ranges[i]);
        return 0;
      }

      if (frame.ranges[i] < min_range)
      {
        min_range = frame.ranges[i];
        min_angle = _angles[i];
      }
    }
    printf("min: %.2f, %.2f\n", min_angle, min_range);
    return 6;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathfinder");
  ros::NodeHandle node;

  static ros::Publisher pub = node.advertise<std_msgs::UInt16>("/mcu/ctl", 1);
  static Pathfinder pathfinder{};
  auto scan_cb = [](sensor_msgs::LaserScan const& frame) {
    auto rps = pathfinder.update_frame(frame);
    std_msgs::UInt16 msg{};
    msg.data = (uint16_t)rps << 8;
    pub.publish(msg);
  };

  using ScanCB = void (*)(sensor_msgs::LaserScan const&);
  auto sub = node.subscribe("/scan", 1, (ScanCB)scan_cb);

  ros::spin();
  return 0;
}
