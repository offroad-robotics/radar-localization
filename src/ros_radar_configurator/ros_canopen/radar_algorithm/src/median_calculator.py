#!/usr/bin/env python
import rospy
#from sklearn import DBSCAN
from pb_msgs.msg import ClusterRadar
from pb_msgs.msg import dbscanOutput
from pb_msgs.msg import individualRadar
from pb_msgs.msg import reflectors
from pb_msgs.msg import medianLatDistOverTime
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rospy.numpy_msg import numpy_msg
import numpy as np
from sklearn.cluster import DBSCAN

median_pub = rospy.Publisher('/median', medianLatDistOverTime, queue_size=10)


global array 
array = np.array([])

#------CHANGE temp_np to NOT include data.target_id---------
#------target_id sometimes not in order!!!!!! THIS IS A HARDWARE ISSUE!!!!-------
shared_list = np.empty((0,3), dtype=np.float32)
max_length = 24
lock = False
reflector_array = np.array([])
mean_array = np.array([])
def callback(data):
    global reflector_array
    global mean_array
    global mean
    reflector_list = (data.reflectors)
    #print(data)
    length = (len(data.reflectors)) #number of waypoints in of reflector path
    #print(length)
    for i in range(0,length):  #caculate median at one time step
        x_pos = reflector_list[i].lat_dist #get lat dist of all reflectors and append them to a np array
        reflector_array = np.append(reflector_array,x_pos)
    #print(reflector_array)
    mean = np.nanmean(reflector_array) #mean at one time step
    #print("mean:", mean)
    append_timesteps(mean) 
    reflector_array = np.array([]) #empty array to start fresh for next time step


#for i in range(5): #calculate median of the medians from 5 time step
# mean_array = np.append(mean_array, mean)
# print(mean_array)
# # MEDIAN = np.median(mean_array)
# # print(MEDIAN)
# # mean_array = np.array([])


#sliding window approach 
def append_timesteps(data):
    median = medianLatDistOverTime()
    global array
    # for i in range(0,5):
    #     i=i+1
    #     array = np.append(array, data)
    array = np.append(array, data)
    if (array.size > 4):
        #print("mean x dist of reflectors over past 5 timesteps: ", array)
        median = np.nanmedian(array)
        median_pub.publish(median)
        #print("median", median)
        array = array[1:]  #pop 0th element off arrray, and shift elements 1-4 1 position left
        #print("array of means post pop", array)
        


def average(data):
    avg = sum(data)/len(data)
    return avg

def listener():
    rospy.init_node('median_calculator', anonymous=True)
    rospy.Subscriber('/reflectors', reflectors, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
    

