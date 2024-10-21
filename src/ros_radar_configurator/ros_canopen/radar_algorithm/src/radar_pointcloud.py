#!/usr/bin/env python
import rospy
#from sklearn import DBSCAN
from pb_msgs.msg import ClusterRadar
from pb_msgs.msg import dbscanOutput
from pb_msgs.msg import individualRadar
from pb_msgs.msg import reflectors
from pb_msgs.msg import medianLatDistOverTime
from pb_msgs.msg import reflectorCount
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rospy.numpy_msg import numpy_msg
import numpy as np
from sklearn.cluster import DBSCAN
import copy


radar_image_pub = rospy.Publisher('radarImage', reflectors, queue_size=10)  #returns x,y pair of all detected objects at once





#------CHANGE temp_np to NOT include data.target_id---------
#------target_id sometimes not in order!!!!!! THIS IS A HARDWARE ISSUE!!!!-------
shared_list = np.empty((0,3), dtype=np.float32)
max_length = 24
lock = False
medianAsync = 0.0

def callback(data):
    global shared_list
    global lock

    a = dbscanOutput()
    # rospy.loginfo("long dist is: %f" % (data.longitude_dist))
    # rospy.loginfo("lat dist is: %f" % (data.lateral_dist))
    # rospy.loginfo("target id: %d" % (data.target_id))
    if (lock):
        if (len(shared_list) < max_length):
            #print(data.target_id)
            #a.target_id = data.target_id
            temp_np = np.array([data.target_id, data.lateral_dist, data.longitude_dist], dtype=np.float32)
            
            #rospy.loginfo('if: %s' % temp_np)
            #shared_list = np.concatenate((shared_list, temp_np), axis=1)
            #shared_list = np.append((shared_list, temp_np), axis=1)
            shared_list = np.append(shared_list,temp_np)
            #rospy.loginfo("in if statement %s" % shared_list)
            temp_np=np.array([])
           # print(len(shared_list))
            #print(shared_list)
        else:
            lock = False
            input_list = np.reshape(shared_list, (-1, 3))  #8x3 array where each row is one target, column is target id, lat, long
            #rospy.loginfo("else statement %s" % input_list)
            shared_list = np.array([])
           # a.longitude_dist = data.longitude_dist
            #a.lateral_dist = data.lateral_dist
            #cluster_labels = run_dbscan(input_list, a)
            #print(input_list)
            #print(cluster_labels)
            full_array = np.c_[input_list]
            #print(np.where(full_array[:,3] == 0))
            #print("full array:", full_array)
            publish_individual(full_array)
            #print(reflectors)
        #publish the y_pred numpy array as a custom ros message called dbscanOuput
    else:
        if(data.target_id ==0):
            temp_np = np.array([data.target_id, data.lateral_dist, data.longitude_dist], dtype=np.float32)
            shared_list = np.append(shared_list,temp_np)
            lock = True
        else:
            pass



#publish information for one radar return at a time
def publish_individual(full_array):
    image = reflectors()
    for radar_object in full_array:
        #print(radar_object)
        temp = individualRadar()
        temp.lat_dist = radar_object[1]
        temp.long_dist = radar_object[2]
        temp.id = radar_object[0]
        image.reflectors.append(temp)
    radar_image_pub.publish(image)
        




def listener():
    rospy.init_node('radar_pointcloud', anonymous=True)
    rospy.Subscriber('/cluster_decoded_messages', numpy_msg(ClusterRadar), callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

