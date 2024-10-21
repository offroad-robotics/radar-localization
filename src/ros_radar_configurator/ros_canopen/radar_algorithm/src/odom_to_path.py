#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomToPath:
    def __init__(self):
        self.path_pub = rospy.Publisher('~path', Path, latch=True, queue_size=10)
        
        self.odom_sub = rospy.Subscriber('/husky_a1/odometry/filtered', Odometry, self.odom_cb, queue_size=10)
        self.path = Path()

    def odom_cb(self, msg):
        cur_pose = PoseStamped()
        cur_pose.header = msg.header
        cur_pose.pose = msg.pose.pose
        self.path.header = msg.header
        self.path.poses.append(cur_pose)
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    rospy.init_node('odom_to_path')
    odom_to_path = OdomToPath()
    rospy.spin()