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
  std::vector<float> _angles;
  std::vector<float> _filtered;
  static constexpr bool lidar_steering true;

public:
  Pathfinder() {}

  CtlMsg update_frame(sensor_msgs::LaserScan const& frame)
  {
    if ((_angles.size() == 0) || (_angles[0] != frame.angle_min[0]))
    {
      _angles.push_back(frame.angle_min[0]);
      for (size_t i = 1; i < frame.ranges.size(); i++)
        _angles.push_back(_angles.back() + frame.angle_increment[0]);
    }

    assert(_angles.size() == frame.ranges.size());

    if constexpr (lidar_steering)
    {
      for (size_t i = 0; i < _angles.size(); i++)
      {
        _filtered.push_back(frame.ranges[i]);
      }

      // Safety radius for car
      static constexpr float safety = 0.500 / 2.0;

      for (int i = 0; i < (int)_angles.size(); i++)
      {
        float r = frame.ranges[i];

        // Ensures distant points are filtered
        if (r > 10)
        {
          r = 10.0;
        }

        auto const t = atan2(safety, r);

        int const d_idx = (int)((t / frame.angle_increment[0]) * 2);
        auto const lower = std::max(0, i - d_idx);
        auto const upper = std::min((int)_angles.size(), i + d_idx);

        for (int j = lower; j < upper; j++)
        {
          if (_filtered[i] > frame.ranges[j])
            _filtered[i] = frame.ranges[j];
        }
      }

      // Selection process
      auto const path = std::max_element(_filtered.begin(), _filtered.end()) -
        _filtered.begin();

      // The selection for angles, ranges etc is where the index is [path]
      printf(
        "index \t angle \t\t range \t filtered \n%2d \t %.3f \t %.3f \t %.3f\n",
        path,
        _angles[path],
        frame.ranges[path],
        _filtered[path]);

      int8_t wheel_angle = round(_angles[path] * 57.30);

      wheel_angle = std::clamp(wheel_angle, -40, 40);

      // Stop when chosen path's range is <1m
      if (frame.ranges[path] < 1.0)
      {
        printf("stop: %.3f, %.3f\n", _angles[path], frame.ranges[path]);
        return CtlMsg{0, 0};
      }
      else
      {
        printf("go: %d, %.3f\n", wheel_angle, frame.ranges[path]);
        return CtlMsg{6, wheel_angle};
      }
    }
    else
    {
      float min_angle = 0.0;
      float min_range = frame.range_max[0];
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
