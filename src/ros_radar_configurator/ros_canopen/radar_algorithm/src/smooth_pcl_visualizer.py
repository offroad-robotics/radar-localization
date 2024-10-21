#!/usr/bin/env python
import rospy

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from pb_msgs.msg import filteredReflectors
# to deal with TF issue in rviz, run all topics in rosbag
path_pub = rospy.Publisher('kfPath', Path, latch=True, queue_size=10)

def callback(data):
    pcl = PointCloud()
    path = Path()
    path.header.frame_id = "/radar"
    
    list = data
    #filling pointcloud header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "/radar"
    pcl.header = header
    #filling some points
        #temp.lat_dist = radar_object[0]
    for i in range((len(data.reflectors))):
        pose = PoseStamped()
        point = Point32()
        point.x = list.reflectors[i].long_dist
        point.y = list.reflectors[i].lat_dist
        pcl.points.append(point)
        pointcloud_publisher.publish(pcl)
        pose.pose.position.x = list.reflectors[i].long_dist
        pose.pose.position.y = list.reflectors[i].lat_dist
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0
        path.poses.append(pose)
    
    path_pub.publish(path)


if __name__ == '__main__':
    '''
    Publishes example pointcloud
    '''
    rospy.init_node('filtered_pcl_vis')
    rospy.Subscriber('trackedreflectors', filteredReflectors, callback)

    pointcloud_publisher = rospy.Publisher("/tracks", PointCloud, queue_size=10)
    rospy.loginfo("pcl_publish_example")
    rospy.spin()
    #giving some time for the publisher to register
    #rospy.sleep(0.5)
    #declaring pointcloud
    #exit. we are done!