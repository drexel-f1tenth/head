# head
Main Control Unit (TX2)

## Build Instructions
```
catkin_make --use-ninja -DCMAKE_BUILD_TYPE=Release
```

## Individual node startup

- `source devel/setup.bash`
- `roscore`
- `rosrun urg_node urg_node _ip_address:=${HOKUYO_DEV}`
- `rosrun pathfinder pathfinder`

## Managing the systemd services

The systemd services, in `service/`, allow all necessary components to autostart then the TX2 it booted up. Each service manages an individual ROS node, except for the `racecar` service. The `racecar` service is used to manage the other associated services as a group.

### Installing the services

- `sudo cp service/*.service /etc/systemd/system/`
- `sudo cp service/*.sh /usr/local/sbin/`
- `sudo systemctl daemon-reload`
- `sudo systemctl enable racecar roscore lidar`

### Useful commands

- Check status: `sudo systemctl status racecar roscore lidar`
- View logs: `journalctl -e`
- Stop a service: `sudo systemctl stop ${service-name}`
- Start a service: `sudo systemctl start ${service-name}`
- Restart a service: `sudo systemctl restart ${service-name}`
- Disable a service: `sudo systemctl mask ${service-name}`

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
