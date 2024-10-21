#!/usr/bin/env python


#custom listener employs dbscan on the noisy pcl

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
from std_msgs.msg import Header

pub1 = rospy.Publisher('dbscanOutput', dbscanOutput, queue_size=10)

pub = rospy.Publisher('individualRadar', individualRadar, queue_size=10)
pub2 = rospy.Publisher('individualReflectors', individualRadar, queue_size=10)
path_pub = rospy.Publisher('reflectorPath', Path, latch=True, queue_size=10)
road_pub = rospy.Publisher('roadPath', Path, latch=True, queue_size=10)
MedianfilteredPath_Pub = rospy.Publisher('medianFilteredReflectorPath', Path, latch=True, queue_size=10)

reflector_count_pub = rospy.Publisher('numReflectors', reflectorCount, queue_size=10)

reflector_pub = rospy.Publisher('dbscanimage', reflectors, queue_size=10)

radar_image_pub = rospy.Publisher('clustered_pcl', reflectors, queue_size=10)  #returns x,y pair of all detected objects at once

median_filtered_reflector_pub = rospy.Publisher('/medianFilteredReflectors', reflectors, queue_size=10)




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
            #print(len(shared_list))
            #print(shared_list)
        else:
            lock = False
            input_list = np.reshape(shared_list, (-1, 3))  #8x3 array where each row is one target, column is target id, lat, long
            #rospy.loginfo("else statement %s" % input_list)
            shared_list = np.array([])
           # a.longitude_dist = data.longitude_dist
            #a.lateral_dist = data.lateral_dist
            cluster_labels = run_dbscan(input_list, a)
            #print(input_list)
            #print(cluster_labels)
            full_array = np.c_[input_list, cluster_labels.transpose()]
            #print("check for dbscan label", full_array)
            #print(np.where(full_array[:,3] == 0))
            #print("full array:", full_array)
            publish_individual(full_array)
            publish_reflectors(full_array)
            #print(reflectors)
        #publish the y_pred numpy array as a custom ros message called dbscanOuput
    else:
        if(data.target_id ==0):
            temp_np = np.array([data.target_id, data.lateral_dist, data.longitude_dist], dtype=np.float32)
            shared_list = np.append(shared_list,temp_np)
            lock = True
        else:
            pass



def run_dbscan(shared_list, a):
    #drop target_id feature from shared list
    #2.5
    db = DBSCAN(eps=1.83, min_samples=3)
    input = np.delete(shared_list, 0, 1) #remove target_id feature from input feature vector to dbscan
    #print(input) 
    db.fit(input)
    y_pred = db.fit_predict(shared_list) # associates a 0, 1 or -1 for each data point for clustering
    #print(type(y_pred))
    a.clusterLabels=y_pred
    pub1.publish(a)
    return(y_pred)

#publish information for one radar return at a time
def publish_individual(full_array):
    b =individualRadar()
    image = reflectors()
    for radar_object in full_array:
        #print(radar_object)
        temp = individualRadar()
        b.lat_dist = radar_object[1]
        temp.lat_dist = radar_object[1]
        b.long_dist = radar_object[2]
        temp.long_dist = radar_object[2]
        b.id = radar_object[0]
        temp.id = radar_object[0]
        b.label = radar_object[3]
        temp.label = radar_object[3]
        pub.publish(b)
        image.reflectors.append(temp)
    radar_image_pub.publish(image)
        
        #print(b)
#-----------------------original working function-------------
# def publish_reflectors(full_array):  #full array is the 8x4 array of radar returns
    
#     mask = (full_array[:,3] == 0) #array of booleans
#     reflector_array = full_array[mask,:]
#     print(reflector_array) #slice of radar object array where the cluster label is equal to 0
#     num_reflectors= ((np.where(mask == True)[0]).size) # number of reflectors
#     c =reflectors()
#     path = Path()
#     pathRoad = Path()
#     path.header.frame_id = "/husky_a1_tf/radar"
#     pathRoad.header.frame_id = "/husky_a1_tf/radar"

#     for reflector in reflector_array:
#         individual_radar = individualRadar()
#         pose = PoseStamped()
#         poseRoad = PoseStamped()
#         individual_radar.id = reflector[0]
#         individual_radar.lat_dist=reflector[1]
#         individual_radar.long_dist=reflector[2]
#         individual_radar.label= reflector[3]
#         c.reflectors.append(individual_radar)
    
#         pose.pose.position.x = reflector[2]
#         pose.pose.position.y = reflector[1]
#         pose.pose.position.z = 0
#         pose.pose.orientation.x = 0
#         pose.pose.orientation.y = 0
#         pose.pose.orientation.z = 0
#         pose.pose.orientation.w = 0
#         path.poses.append(pose)

#         poseRoad.pose.position.x = reflector[2] 
#         poseRoad.pose.position.y = reflector[1] + 2
#         poseRoad.pose.position.z = 0
#         poseRoad.pose.orientation.x = 0
#         poseRoad.pose.orientation.y = 0
#         poseRoad.pose.orientation.z = 0
#         poseRoad.pose.orientation.w = 0
#         pathRoad.poses.append(poseRoad)
        
#     path_pub.publish(path)
#     road_pub.publish(pathRoad)
#     reflector_pub.publish(c)
#-------------------------------------------------------------------------

def callback2(data):
    global medianAsync
    medianAsync = data.median



def publish_reflectors(full_array):  #full array is the 8x4 array of radar returns
    count = reflectorCount()
    global medianAsync
    mask = (full_array[:,3] == 0) #array of booleans
    reflector_array = full_array[mask,:]
    #print(reflector_array) #slice of radar object array where the cluster label is equal to 0
    num_reflectors= ((np.where(mask == True)[0]).size) # number of reflectors
    #publish num_reflectors
    count.reflectorCount = num_reflectors
    reflector_count_pub.publish(count)
    c =reflectors() #will publish all reflectors predicted by dbscan
    filteredReflectors = reflectors() #will publish only the dbscan predictions that satisfy the median filter threshold
    
    # Create a header object with the current time
    c.header = Header()
    c.header.stamp = rospy.Time.now()
    c.header.frame_id = "/radar"  # Set the frame ID


    path = Path()
    pathRoad = Path()
    filteredPath = Path()

    path.header.frame_id = "/odom"
    pathRoad.header.frame_id = "/radar"
    filteredPath.header.frame_id = "/radar"


    for reflector in reflector_array:
        individual_radar = individualRadar()
        
        pose = PoseStamped()
        poseRoad = PoseStamped()
        poseFiltered = PoseStamped()

        individual_radar.id = reflector[0]
        individual_radar.lat_dist=reflector[1]
        #print("lat dist sanity check", individual_radar.lat_dist)
        individual_radar.long_dist=reflector[2]
        individual_radar.label= reflector[3]
        c.reflectors.append(individual_radar)
        
        #apply median filter with threshold of 1.5 m
        #if error b/w detected point and median is small, we append it, otherwise, skip that detection 
        if abs(individual_radar.lat_dist - medianAsync) < 1.5:
            #print("filtering with median filter")
            filteredReflectors.reflectors.append(individual_radar)
            poseFiltered.pose.position.x = reflector[2]
            poseFiltered.pose.position.y = reflector[1]
            poseFiltered.pose.position.z = 0
            poseFiltered.pose.orientation.x = 0
            poseFiltered.pose.orientation.y = 0
            poseFiltered.pose.orientation.z = 0
            poseFiltered.pose.orientation.w = 0
            filteredPath.poses.append(pose)


            poseRoad.pose.position.x = reflector[2] 
            poseRoad.pose.position.y = reflector[1] + 0.8
            poseRoad.pose.position.z = 0
            poseRoad.pose.orientation.x = 0
            poseRoad.pose.orientation.y = 0
            poseRoad.pose.orientation.z = 0
            poseRoad.pose.orientation.w = 0
            pathRoad.poses.append(poseRoad)

        else:
            pass
    
        pose.pose.position.x = reflector[2]
        pose.pose.position.y = reflector[1]
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0
        path.poses.append(pose)

        
        
    path_pub.publish(path)
    road_pub.publish(pathRoad)
    reflector_pub.publish(c)
    median_filtered_reflector_pub.publish(filteredReflectors)
    MedianfilteredPath_Pub.publish(filteredPath)




# def median_filter(data):
#     median = data
#     print(median)


def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber('/cluster_decoded_messages', numpy_msg(ClusterRadar), callback)
    rospy.Subscriber('/median', medianLatDistOverTime, callback2, queue_size=5)
    rospy.spin()


if __name__ == '__main__':
    listener()

