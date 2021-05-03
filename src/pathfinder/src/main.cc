#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>
#include <vector>

struct CtlMsg
{
  int8_t throttle;
  int8_t steering;

  CtlMsg(int8_t throttle_, int8_t steering_)
  : throttle{throttle_}, steering{steering_}
  {}

  std_msgs::UInt16 serialize() const
  {
    std_msgs::UInt16 msg;
    msg.data = ((uint16_t)throttle << 8) | ((uint16_t)steering & 0xff);
    return msg;
  }
};

class Pathfinder
{
  std::vector<float> _angles;
  std::vector<float> _filtered;
#define lidar_steering

public:
  Pathfinder() {}

  CtlMsg update_frame(sensor_msgs::LaserScan const& frame)
  {
    float _ranges = frame.ranges;
    float _angle_min = frame.angle_min;
    float _angle_max = frame.angle_max;
    float _angle_increment = frame.angle_increment;
    float _range_max = frame.range_max;
    float _range_min = frame.range_min;

    if ((_angles.size() == 0) || (_angles[0] != _angle_min[0]))
    {
      _angles.push_back(_angle_min[0]);
      for (size_t i = 1; i < _ranges.size(); i++)
        _angles.push_back(_angles.back() + _angle_increment[0]);
    }

    assert(_angles.size() == _ranges.size());

#ifdef lidar_steering
    for (size_t i = 0; i < _angles.size(); i++)
    {
      _filtered.push_back(_ranges[i]);
    }

    // Safety radius for car
    float safety = 0.500 / 2.0;

    for (int i = 0; i < (int)(_angles.size()); i++)
    {
      float _r = _ranges[i];

      // Ensures distant points are filtered
      if (_r > 10)
      {
        _r = 10.0;
      }

      float t = atan2(safety, _r);

      int d_idx = (int)(t / _angle_increment[0]) * 2;
      int lower = std::max(0, i - d_idx);
      int upper = std::min((int)(_angles.size()), i + d_idx);

      for (int j = lower; j < upper; j++)
      {
        if (_filtered[i] > _ranges[j])
          _filtered[i] = _ranges[j];
      }
    }

    // Selection process
    float max_filt_range = {0};
    int path = {0};
    for (size_t i = 0; i < _filtered.size(); i++)
    {
      if (_filtered[i] > max_filt_range)
      {
        max_filt_range = _filtered[i];
        path = i;
      }
    }

    // _All the path index variables are the selection
    printf(
      "index \t angle \t\t range \t filtered \n%2d \t %.3f \t %.3f \t %.3f\n",
      path,
      _angles[path],
      _ranges[path],
      _filtered[path]);

    int8_t wheel_angle = round(_angles[path] * 57.30);

    if (wheel_angle > 40)
    {
      wheel_angle = 40;
    }
    else if (wheel_angle < -40)
    {
      wheel_angle = -40;
    }

    // Stop when chosen path's range is <1m
    if (_ranges[path] < 1.0)
    {
      printf("stop: %.3f, %.3f\n", _angles[path], _ranges[path]);
      return CtlMsg{0, 0};
    }
    else
    {
      printf("go: %d, %.3f\n", wheel_angle, _ranges[path]);
      return CtlMsg{6, wheel_angle};
    }

#else

    float min_angle = 0.0;
    float min_range = _range_max[0];
    for (size_t i = 0; i < _ranges.size(); i++)
    {
      // If the angle is outside of +/- 0.2 rads then break that loop iteration
      if (fabs(_angles[i]) > 0.2)
        continue;

      if (_ranges[i] < 1.0)
      {
        printf("stop: %.3f, %.3f\n", _angles[i], _ranges[i]);
        return CtlMsg{0, 0};
      }

      // If detected range is less than maximum possible range
      // Store as the "new" minimum range observed
      if (_ranges[i] < min_range)
      {
        min_range = _ranges[i];
        min_angle = _angles[i];
      }
    }
    printf("min: %.3f, %.3f\n", min_angle, min_range);
    return CtlMsg{6, 0};
#endif
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathfinder");
  ros::NodeHandle node;

  static ros::Publisher pub = node.advertise<std_msgs::UInt16>("/mcu/ctl", 1);
  static Pathfinder pathfinder{};
  auto scan_cb = [](sensor_msgs::LaserScan const& frame) {
    auto const msg = pathfinder.update_frame(frame);
    pub.publish(msg.serialize());
  };
  
  // pathfinder.update_frame();
  using ScanCB = void (*)(sensor_msgs::LaserScan const&);
  auto sub = node.subscribe("/scan", 1, (ScanCB)scan_cb);

  ros::spin();
  return 0;
}
