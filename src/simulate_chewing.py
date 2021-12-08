#!/usr/bin/env python

import time
import rospy
import numpy as np
from std_msgs.msg import Float64

CMD_OPEN = 1.0
CMD_CLOSE = 0.2
CMD_TOPIC = 'mouth_cmd' # The topic to publish the mouth cmd to

# This test script commands the mouth to change its position to CMD_TARGET
if __name__ == '__main__':
  rospy.init_node('mouth_client', anonymous=True)

  # Create a publisher of commands
  cmd_pub = rospy.Publisher(CMD_TOPIC, Float64, queue_size=1)
  rate = rospy.Rate(100)

  # Send the command
  while not rospy.is_shutdown():
    msg = Float64()
    cmd = np.linspace(CMD_CLOSE, CMD_OPEN, num=50)
    for mouth_pos in reversed(cmd):
        msg.data = mouth_pos
        cmd_pub.publish(msg)
        rate.sleep()
    for mouth_pos in cmd[1:-1]:
        msg.data = mouth_pos
        cmd_pub.publish(msg)
        rate.sleep()
