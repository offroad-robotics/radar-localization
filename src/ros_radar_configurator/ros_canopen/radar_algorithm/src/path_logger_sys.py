#!/usr/bin/env python
import rospy
import csv
import datetime
import os
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from pb_msgs.msg import reflectors
from visualization_msgs.msg import Marker
# x is longitudinal direction from robot
# the data is published in ros according to increasing longitudinal distance 
#maybe the points should go thru the odom frame transform
def callback(data):
    global writer
    list = data    
    timestamp= rospy.Time().now().to_sec()
    to_write = []
    to_write.append(timestamp)
    
    for i in range(8):
        if (list.reflectors[i].label == 0): # //if DBSCAN outputs reflector prediction, color it red
            to_write.append(list.reflectors[i].long_dist)
            to_write.append(list.reflectors[i].lat_dist)
        else:
            to_write.append('')
            to_write.append('')        
    print(to_write)
    writer.writerow(to_write)

# This generates a unique file name with the timestamp
def generate_log_file_name(path):
    time = datetime.datetime.now().strftime("%Y-%m-%d_%I-%M-%S")
    name = 'path_log_' + time + '.csv'
    full_path = os.path.join(path, name)
    return full_path

def logger():
    global writer
    path = 'path_csv_logs/feb2-navigation-testing/1speed/CurveRight'
    name = generate_log_file_name(path)
    file = open(name, 'w')
    writer = csv.writer(file)
    writer.writerow(["timestamp", "X1", "Y1", "X2", "Y2", "X3", "Y3", "X4", "Y4", "X5", "Y5", "X6", "Y6","X7", "Y7", "X8", "Y8"])
    rospy.init_node('path_logger', anonymous=True)
    rospy.Subscriber('dbscanimage', reflectors, callback)
    rospy.spin()


if __name__ == '__main__':
    logger()
