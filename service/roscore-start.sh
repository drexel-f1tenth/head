. /opt/ros/melodic/setup.sh
roscore &
while ! echo exit | nc localhost 11311 >/dev/null; do
  sleep 1
done
