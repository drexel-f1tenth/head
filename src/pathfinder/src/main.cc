#include <algorithm>
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
  static constexpr bool lidar_steering = true;
  static constexpr float rad_to_deg = 180.0 / M_PI;
  static constexpr float deg_to_rad = M_PI / 180.0;
  static constexpr float angle_max = 60.0 * deg_to_rad;

  std::vector<float> _angles;
  std::vector<float> _filtered;
  size_t _range_start = 0;

public:
  Pathfinder() {}

  CtlMsg update_frame(sensor_msgs::LaserScan const& frame)
  {
    if (_angles.size() == 0)
    {
      auto angle = (double)frame.angle_min;
      for (size_t i = 1; i < frame.ranges.size(); i++)
      {
        angle += frame.angle_increment;
        if ((-angle_max <= angle) && (angle <= angle_max))
        {
          if (_range_start == 0)
            _range_start = i;

          _angles.push_back(angle);
        }
      }
      _filtered.resize(_angles.size());
    }

    if constexpr (lidar_steering)
    {
      // TODO: avoid this copy
      _filtered.clear();
      auto const* ranges = &frame.ranges[_range_start];
      for (size_t i = 0; i < _angles.size(); i++)
        _filtered.push_back(ranges[i]);

      static constexpr float safety = 0.500 / 2.0;

      for (int i = 0; i < (int)_angles.size(); i++)
      {
        auto const r = std::min<float>(ranges[i], 10.0);
        auto const t = atan2(safety, r);
        auto const d_idx = (int)((t / frame.angle_increment) * 2);
        auto const lower = std::max(0, i - d_idx);
        auto const upper = std::min((int)_angles.size(), i + d_idx);
        for (int j = lower; j < upper; j++)
        {
          if (_filtered[i] > ranges[j])
            _filtered[i] = ranges[j];
        }
      }

      // Selection process
      auto const path = std::max_element(_filtered.begin(), _filtered.end()) -
        _filtered.begin();

      auto const steering_angle =
        std::clamp<int8_t>(-round(_angles[path] * rad_to_deg), -40, 40);

      printf(
        "angle: %d, range: %.2f, filtered: %.2f\n",
        steering_angle,
        ranges[path],
        _filtered[path]);

      int8_t const throttle = (ranges[path] < 1.0) ? 0 : 6;
      return CtlMsg{throttle, steering_angle};
    }
    else
    {
      float min_angle = 0.0;
      float min_range = frame.range_max;
      for (size_t i = 0; i < frame.ranges.size(); i++)
      {
        // If the angle is outside of +/- 0.2 rads then break that loop
        // iteration
        if (fabs(_angles[i]) > 0.2)
          continue;

        if (frame.ranges[i] < 1.0)
        {
          printf("stop: %.3f, %.3f\n", _angles[i], frame.ranges[i]);
          return CtlMsg{0, 0};
        }

        // If detected range is less than maximum possible range
        // Store as the "new" minimum range observed
        if (frame.ranges[i] < min_range)
        {
          min_range = frame.ranges[i];
          min_angle = _angles[i];
        }
      }
      printf("min: %.3f, %.3f\n", min_angle, min_range);
      return CtlMsg{6, 0};
    }
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

  using ScanCB = void (*)(sensor_msgs::LaserScan const&);
  auto sub = node.subscribe("/scan", 1, (ScanCB)scan_cb);

  ros::spin();
  return 0;
}
