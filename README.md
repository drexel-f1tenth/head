# head
Main Control Unit (TX2)

## Installation
```
catkin_make --use-ninja -DCMAKE_BUILD_TYPE=Release
```

## Individual node startup
- `source devel/setup.bash`
- `roscore`
- `rosrun urg_node urg_node _ip_address:=${HOKUYO_DEV}`
- `rosrun pathfinder pathfinder`

## Additional notes
- [sensor_msgs/LaserScan Message](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)
- It may be necessary to toggle the eth0 interface using `ifdown eth0` and `ifup eth0`
