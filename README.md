# head
Main Control Unit (TX2)

## Build Instructions
```
catkin_make --use-ninja -DCMAKE_BUILD_TYPE=Release
```

## Manual node startup

- `source devel/setup.bash`
- `roscore`
- `rosrun urg_node urg_node _ip_address:=${HOKUYO_DEV}`
- `rosrun pathfinder pathfinder`

## MCU control message format

The `mcu/ctl` topic uses the ROS UInt16 message type where the throttle and steering directions are encoded as packed Int8 values.
- serialization:
  ```
  serialized = ((uint16_t)throttle << 8) | ((uint16_t)steering & 0xff);
  ```
- deserialization:
  ```
  throttle = (int8_t)(serialized.data >> 8);
  steering = (int8_t)(serialized.data & 0xff);
  ```

## Tools

- `scripts/lidar-debug.py`: Generate a polar plot from the output of the LiDAR (assuming frames are published to the `/scan` topic)
- `mcu_pub`: Publish a control message to the MCU (on the `mcu/ctl` topic)
  - Run with `rosrun mcu_pub mcu_pub <throttle> <steering>`

## Additional notes

- [sensor_msgs/LaserScan Message](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)
- If you want to connect to the internet, it may be necessary to toggle the `eth0` interface using `ifdown eth0` and `ifup eth0`. The `eth0` interface is used for communication with the LiDAR, but the network manager on the TX2 prioritizes it over WiFi interfaces even though it is not connected to a network.
