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
  static constexpr float safety_radius = 0.500 / 2.0;
  static constexpr float rad_to_deg = 180.0 / M_PI;
  static constexpr float deg_to_rad = M_PI / 180.0;
  static constexpr float angle_max = 60.0 * deg_to_rad;

  std::vector<float> _angles;
  std::vector<float> _filtered;
  size_t _range_start = 0;
  int8_t _steering_angle = 0;

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

    auto const* ranges = &frame.ranges[_range_start];
    std::copy(&ranges[0], &ranges[_angles.size()], _filtered.begin());

    size_t stop_counter = 0;
    for (int i = 0; i < (int)_angles.size(); i++)
    {
      if (
        (ranges[i] < 0.5) ||
        ((abs(_angles[i]) < (5.0 * deg_to_rad)) && (ranges[i] <= 1.0)))
      {
        // printf("angle: %d, range: %.2f\n", _steering_angle, ranges[i]);
        return CtlMsg{0, _steering_angle};
      }

      auto const r = std::min<float>(ranges[i], 10.0);
      auto const t = atan2(safety_radius, r);
      auto const d_idx = (int)std::ceil(t / frame.angle_increment);
      auto const lower = std::max(0, i - d_idx);
      auto const upper = std::min((int)_angles.size(), i + d_idx);
      auto const local_min = *std::min_element(&ranges[lower], &ranges[upper]);
      for (int j = lower; j < upper; j++)
        _filtered[j] = std::min(_filtered[j], local_min);
    }

    auto const path =
      std::max_element(_filtered.begin(), _filtered.end()) - _filtered.begin();

    _steering_angle =
      std::clamp<int8_t>(-round(_angles[path] * rad_to_deg), -40, 40);

    // printf(
    //   "angle: %d, range: %.2f, filtered: %.2f\n",
    //   _steering_angle,
    //   ranges[path],
    //   _filtered[path]);

    return CtlMsg{6, _steering_angle};
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
