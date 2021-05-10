import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

def cb(data):
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='polar')
  angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
  ranges = np.clip(data.ranges, None, 6)
  ax.plot(angles, ranges)
  ax.set_theta_zero_location('N')
  plt.tight_layout()
  plt.savefig('lidar-debug.png', format='png', dpi=200)
  print("output: lidar-debug.png")
  rospy.signal_shutdown("done")

sub = rospy.Subscriber('/scan', LaserScan, cb)
rospy.init_node('lidar_debug', anonymous=True)
rospy.spin()
