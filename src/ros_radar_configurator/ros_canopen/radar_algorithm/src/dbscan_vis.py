#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from pb_msgs.msg import reflectors
from visualization_msgs.msg import Marker
# to deal with TF issue in rviz, run all topics in rosbag
# path_pub = rospy.Publisher('kfPath', Path, latch=True, queue_size=10)
path_pub = rospy.Publisher('filteredPath', Path, latch=True, queue_size=10)


def callback(data):
    marker = Marker()
    marker.header.frame_id = "/radar"
    path = Path()
    path.header.frame_id = "/radar"
    marker.type = marker.SPHERE
    # path = Path()

    
    list = data
    #print("list", list)
    #filling pointcloud header

    #filling some points
        #temp.lat_dist = radar_object[0]
    
    for i in range(8):
        pose = PoseStamped()
        # pose.pose.position.x = 0
        # pose.pose.position.y = 0
        # pose.pose.position.z = 0
        # pose.pose.orientation.x = 0
        # pose.pose.orientation.y = 0
        # pose.pose.orientation.z = 0
        # pose.pose.orientation.w = 0
        marker.action = marker.ADD
        marker.id = i
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.pose.position.x = list.reflectors[i].long_dist
        marker.pose.position.y = list.reflectors[i].lat_dist
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        pose.pose.position.x = list.reflectors[i].long_dist
        pose.pose.position.y = list.reflectors[i].lat_dist    #1m left offset
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0
        if (list.reflectors[i].label == 0): # //if DBSCAN outputs reflector prediction, color it green
            marker.color.r = 0
            marker.color.g = 255
            marker.color.b = 0
            marker.color.a = 0.5
            path.poses.append(pose)
        # else:
        #     marker.color.r = 255
        #     marker.color.g = 0
        #     marker.color.b = 0
        #     marker.color.a = 0.5
        elif(list.reflectors[i].label == 1):
            marker.color.g = 0
            marker.color.a = 0.5
            marker.color.r = 0
            marker.color.b = 255
        elif (list.reflectors[i].label ==-1):
            marker.color.b = 0
            marker.color.a = 0.5
            marker.color.g = 0
            marker.color.r=255
        marker_publisher.publish(marker)
        
           
    path_pub.publish(path)

        


if __name__ == '__main__':
    '''
    Publishes example pointcloud
    '''
    rospy.init_node('dbscan_vis')
    rospy.Subscriber('clustered_pcl', reflectors, callback)

    marker_publisher = rospy.Publisher("/dbscan_vis", Marker, queue_size=10)
    rospy.spin()
    #giving some time for the publisher to register
    #rospy.sleep(0.5)
    #declaring pointcloud
    #exit. we are done!
    print "bye bye..."