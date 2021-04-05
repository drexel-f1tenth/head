#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/UInt16.h>
#include <string>
#include <vector>

struct Msg
{
  int8_t throttle;
  int8_t steering;

  Msg(int8_t throttle_, int8_t steering_)
  : throttle{throttle_}, steering{steering_}
  {}

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "{throttle: " << (int)throttle << ", steering: " << (int)steering
       << "}";
    return ss.str();
  }

  std_msgs::UInt16 serialize() const
  {
    std_msgs::UInt16 msg;
    msg.data = ((uint16_t)throttle << 8) | (uint16_t)steering;
    return msg;
  }
};

static void print_usage(char const* arg0)
{
  std::cout << "Usage: " << arg0
            << " <throttle> <steering>\n"
               "  throttle: Int8\n"
            << "  steering: Int8\n"
            << std::flush;
}

static std::optional<int8_t> parse_arg(char const* input)
{
  try
  {
    auto const value = std::stoi(input);
    if (
      (value < std::numeric_limits<int8_t>::min()) ||
      (std::numeric_limits<int8_t>::max() < value))
      return {};

    return value;
  }
  catch (std::exception const& e)
  {
    return {};
  }
}

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    print_usage(argv[0]);
    return 1;
  }

  auto const throttle = parse_arg(argv[1]);
  if (!throttle)
  {
    std::cout << "invalid throttle: " << argv[1] << "\n\n";
    print_usage(argv[0]);
    return 1;
  }
  auto const steering = parse_arg(argv[2]);
  if (!steering)
  {
    std::cout << "invalid steering: " << argv[2] << "\n\n";
    print_usage(argv[0]);
    return 1;
  }

  Msg const msg(*throttle, *steering);

  ros::init(argc, argv, "mcu_pub");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<std_msgs::UInt16>("/mcu/ctl", 0, true);
  ros::start();

  auto const serialized = msg.serialize();
  std::cout << "sending " << msg.to_string() << std::endl;
  pub.publish(serialized);

  ros::Rate rate(10);
  using clock = std::chrono::steady_clock;
  auto const timeout = clock::now() + std::chrono::milliseconds(300);
  while (node.ok() && (clock::now() < timeout))
    rate.sleep();

  return 0;
}
