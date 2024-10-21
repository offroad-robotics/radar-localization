#!/usr/bin/env python
#this file is used to visualize in rviz the output of the radar (radarImage topic) with path through it
#no dbscan applied here
#shows contrast between smooth and noisy paths after kalman filter applied
import rospy

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from pb_msgs.msg import filteredReflectors
from pb_msgs.msg import reflectors


path_pub = rospy.Publisher('noisyPath', Path, latch=True, queue_size=10)

#currently, the visualization is in the frame of the radar. We can change this to the odom frame, however


def callback(data):
    pcl_odom = PointCloud()
    pcl_radar = PointCloud() 
    path = Path()  #we make a duplicate of this pointcloud in two frames
    path.header.frame_id = "/radar"
    list = data
    #filling pointcloud header
    header_odom = std_msgs.msg.Header()
    header_odom.stamp = rospy.Time.now()
    header_odom.frame_id = "/radar"
    pcl_odom.header = header_odom
    #print(list)
    # header_radar = std_msgs.msg.Header()
    # header_radar.stamp = rospy.Time.now()
    # header_radar.frame_id = "/radar"
    # pcl_radar.header = header_radar

    #filling some points
        #temp.lat_dist = radar_object[0]
    for i in range(8):
        #print(i)
        pose = PoseStamped()
        point = Point32()
        point.x = list.reflectors[i].long_dist
        point.y = list.reflectors[i].lat_dist
        pcl_odom.points.append(point)
        pointcloud_publisher.publish(pcl_odom)
        # pcl_radar.points.append(point)
        # pointcloud_publisher.publish(pcl_radar)
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
    rospy.init_node('radar_image_vis')
    rospy.Subscriber('clustered_pcl', reflectors, callback)

    pointcloud_publisher = rospy.Publisher("/radar_pointcloud", PointCloud, queue_size=10)
    rospy.loginfo("pcl_publish_example")
    rospy.spin()
    #giving some time for the publisher to register
    #rospy.sleep(0.5)
    #declaring pointcloud
    #exit. we are done!
   