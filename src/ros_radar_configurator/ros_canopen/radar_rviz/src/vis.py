#! /usr/bin/env python
import rospy
#from sklearn import DBSCAN
from pb_msgs.msg import ClusterRadar
from pb_msgs.msg import dbscanOutput

import rospy
from visualization_msgs.msg import Marker

a = dbscanOutput()

#marker = Marker()



#------CHANGE temp_np to NOT include data.target_id---------
input1 = dbscanOutput()

def callback(input):
    sphere = Marker()
    marker_pub = rospy.Publisher("/visualization_marker_python", Marker, queue_size = 2)

    sphere.header.frame_id = "/radar"
    sphere.header.stamp = rospy.Time.now()

    sphere.ns = "markers"
    sphere.action = 0

    # sphere.header.stamp = ros::Time();
    # sphere.lifetime = ros::Duration(0.5);

    sphere.id = input.target_id

    sphere.type = sphere

    sphere.scale.x = 0.1
    sphere.scale.y = 0.1
    sphere.scale.z = 0.1
    
    sphere.color.b = 1.0
    sphere.color.a = 1.0

    # if (input1.clusterLabels[0] == 0):

     
    #      sphere.color.r = 1.0
    #      sphere.color.a = 1.0
     
    # else:
    #      if (input1.clusterLabels[0] == 1):

     
    #         sphere.color.g = 1.0
    #         sphere.color.a = 1.0 
     
    #      if(input1.clusterLabels[0] == -1):
    #         sphere.color.b = 1.0
    #         sphere.color.a = 1.0




    sphere.pose.orientation.w = 1.0
    sphere.pose.position.x = input.longitude_dist
    sphere.pose.position.y = input.lateral_dist
    sphere.pose.position.z = 0
    marker_pub.publish(sphere)
    
    labels = access_labels()
    print(labels)


def access_labels():
    pub = rospy.Publisher('dbscanOutput', dbscanOutput, queue_size=10)
    pub.publish(a)

def listener():
    rospy.init_node('custom_vis', anonymous=True)
    rospy.Subscriber('/cluster_decoded_messages', ClusterRadar, callback)
    rospy.Subscriber('dbscanOutput',dbscanOutput, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    listener()