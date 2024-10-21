#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from pb_msgs.msg import cov
from pb_msgs.msg import filteredReflectors
from pb_msgs.msg import filteredReflector
from pb_msgs.msg import reflectors
from scipy.stats import chi2
import numpy as np
import os
import csv
import math
rospy.init_node('covariance_plotter')

import re

#function for reading data from Vicon CSV files
def read_csv_column(file_path, column_name):
    with open(file_path, 'r') as file:
        reader = csv.DictReader(file)
        column_data = []
        for i, row in enumerate(reader):
            if i == 0:
                continue  # Skip the first row (headers)
            value = row[column_name]
            #value = re.sub(r"^['\"]|['\"]$", "", value)  # Remove surrounding quotes
            column_data.append(value)
        return column_data
# Example usage
file_path = '/home/dean/radar-navigation/thesis-groundtruth/straight-half.csv'
#file_path = '/home/dean/radar-navigation/thesis-groundtruth/left-half.csv'
#file_path = '/home/dean/radar-navigation/thesis-groundtruth/right-half.csv'
col1 = 'GT1X'
col2 = 'GT2X'
col3 = 'GT3X'
col4 = 'GT4X'
col5 = 'GT5X'

col6 = 'GT1Y'
col7 = 'GT2Y'
col8 = 'GT3Y'
col9 = 'GT4Y'
col10 = 'GT5Y'

#saving the plots
home_dir = os.path.expanduser("~")
cov1_path = os.path.join(home_dir, "radar-navigation", "plots", "cov1_plot.pdf")
cov2_path = os.path.join(home_dir, "radar-navigation", "plots", "cov2_plot.pdf")
cov3_path = os.path.join(home_dir, "radar-navigation", "plots", "cov3_plot.pdf")
cov4_path = os.path.join(home_dir, "radar-navigation", "plots", "cov4_plot.pdf")
cov5_path = os.path.join(home_dir, "radar-navigation", "plots", "cov5_plot.pdf")
covy_path = os.path.join(home_dir, "radar-navigation", "plots", "covy_plot.pdf")

pcl_path = os.path.join(home_dir, "radar-navigation", "plots", "pcl_plot.pdf")
errorplot_path = os.path.join(home_dir, "radar-navigation", "plots", "error.pdf")

# Create a list to store the data from MOT node
data1 = []
data1neg = []
data2 = []
data2neg =[]
data3 = []
data3neg =[]
data4 = []
data4neg =[]
data5 = []
data5neg =[]
pred_y = []
pred_x = []
pcl_y = []
pcl_x= []
cluster_x=[]
cluster_y=[]


x1 = []
y1 = []
x2 = []
y2 = []
x3 = []
y3 = []
x4 = []
y4 = []
x5 = []
y5 = []

# Create a callback function that will be called every time a new message is received
# for plotting confidence bounds
def callback(msg):
  # positive bounds
  data1.append(msg.cov1)
  data2.append(msg.cov2)
  data3.append(msg.cov3)
  data4.append(msg.cov4)
  data5.append(msg.cov5)
  # negative bounds  
  data1neg.append(msg.cov1*(-1))
  data2neg.append(msg.cov2*(-1))
  data3neg.append(msg.cov3*(-1))
  data4neg.append(msg.cov4*(-1))
  data5neg.append(msg.cov5*(-1))

#callback for storing x,y positiion of five tracked objects
def callback2(msg1):
  global mean
  temp_x=np.array([])
  temp_y = np.array([])
  reflector_list = (msg1.reflectors)
  x1.append(msg1.reflectors[0].long_dist)
  x2.append(msg1.reflectors[1].long_dist)
  x3.append(msg1.reflectors[2].long_dist)
  x4.append(msg1.reflectors[3].long_dist)
  x5.append(msg1.reflectors[4].long_dist)
  y1.append(msg1.reflectors[0].lat_dist*(-1))
  y2.append(msg1.reflectors[1].lat_dist*(-1))
  y3.append(msg1.reflectors[2].lat_dist*(-1))
  y4.append(msg1.reflectors[3].lat_dist*(-1))
  y5.append(msg1.reflectors[4].lat_dist*(-1))
  for i in range(len(reflector_list)):
    pred_x.append(msg1.reflectors[i].lat_dist*(-1))
    pred_y.append(msg1.reflectors[i].long_dist)

#callback for storing entire pointcloud to plot clutter
def callback3(data):
  temp_x=np.array([])
  temp_y = np.array([])
  for i in range(8):
        pcl_x.append(data.reflectors[i].lat_dist*(-1))
        pcl_y.append(data.reflectors[i].long_dist) #from robots frame, longitude is the x direction

def callback4(data):
    reflector_list = (data.reflectors)
    length = (len(data.reflectors)) #number of waypoints in of reflector path
    for i in range(0, length):   #accounts for the possibility of dbscan outputting >5 returns
        cluster_x.append(reflector_list[i].lat_dist*(-1)) 
        cluster_y.append(reflector_list[i].long_dist)
def create_scatter_plot(pcl_x, pcl_y, cluster_x, cluster_y,x1, y1, x2, y2, x3, y3,x4, y4,x5, y5,xlabel, ylabel):
    """
    Creates a scatter plot comparing two things in different colors.
    
    Parameters:
    x1 (list): The x-values for the first set of data points
    y1 (list): The y-values for the first set of data points
    x2 (list): The x-values for the second set of data points
    y2 (list): The y-values for the second set of data points
    xlabel (str): The label for the x-axis
    ylabel (str): The label for the y-axis
    title (str): The title for the plot
    
    Returns:
    None
    """
    # Create the plot
    fig, ax = plt.subplots()
    #plot clutter
    ax.scatter(pcl_x, pcl_y, c='lavender', label='Clutter')
    #plot tracks
    plt.plot(y1,x1, color='red', label="track1")
    plt.plot(y2,x2, color='green', label="track2")
    plt.plot(y3,x3, color='yellow', label="track3")
    plt.plot(y4,x4, color='orange', label="track4")
    plt.plot(y5,x5, color='blue', label="track5")
    

    #for dbscan clustering output (measurements)
    ax.scatter(cluster_x, cluster_y, c='cornflowerblue', label='Reflector Measurements')

    # Add labels and title
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    
    # Add a legend
    ax.legend(loc = 'upper left')
    plt.grid(linewidth = 0.5)
    plt.ylim([-2,12])
    plt.xlim([-8,8])
    plt.savefig(pcl_path)
    # Show the plot
    plt.ioff()
    plt.show()

#function to downsample the Vicon data to match the frequency of the state estimator data
def downsample_gt(gt, est):
    # Calculate the downsampling factor
    downsample_factor = len(gt) // len(est)

    # Downsample the larger list
    downsampled_list1 = gt[::downsample_factor]

    # Adjust the downsampled list length to match the smaller list length
    downsampled_list1 = downsampled_list1[:len(est)]
    return downsampled_list1

#function to plot the ground truth, estimation, error, and confidence bounds
def plot_covariance(data1, est, gt, error1, data1neg, x, cov_path):
    #downsample
    gt = downsample_gt(gt, error1)

    plt.plot(error1, color='orange', label="error")
    #comment out the next two lines to only plot error
    #plt.plot(gt, color='green', marker = 'o', label="ground truth")
    #plt.plot(est, color='orange', marker = 'o', label="estimation")

    plt.ylabel("Longitudinal Range [m]", fontsize=12)
    plt.xlabel("Measurement Number", fontsize=12)
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12)
    data1neg = np.array(data1neg)  # Convert data1neg to a NumPy array

    #calculate 95% confidence from the variance:
    #Calculate standard deviation
    standard_deviation = np.sqrt(data1)
    # Calculate critical value for a 95% confidence interval
    critical_value = 1.96
    # Calculate margin of error
    margin_of_error = critical_value * standard_deviation

    #confidence around 0 with error plotted
    plt.fill_between(x, 0 + margin_of_error, 0 - margin_of_error, color='lightblue', alpha=0.5, label='95% Confidence')    
    plt.plot(x, 0 + margin_of_error, color='blue', linestyle='-', linewidth=1)  # Solid blue line upper bound
    plt.plot(x, 0 - margin_of_error, color='blue', linestyle='-', linewidth=1)  # Solid blue line lower bound
    plt.legend()
    #plt.ylim([-2,2])
    plt.savefig(cov_path)
    plt.ioff()
    plt.show()

    #confidence around estimation with ground truth also plotted
    # plt.fill_between(x, est + margin_of_error, est - margin_of_error, color='lightblue', alpha=0.5, label='95% Confidence')    
    # plt.plot(x, np.array(est) + margin_of_error, color='blue', linestyle='-', linewidth=1)  # Solid blue line upper bound
    # plt.plot(x, np.array(est) - margin_of_error, color='blue', linestyle='-', linewidth=1)  # Solid blue line lower bound
    # plt.legend()
    # plt.savefig(cov_path)
    # plt.ioff()
    # plt.show()

#function to calculate error (in 1D) between estimation and ground truth
def error(list1, list2):
    if len(list1) > len(list2):
        step = len(list1) // len(list2)
        list1 = list1[::step][:len(list2)]
    elif len(list2) > len(list1):
        step = len(list2) // len(list1)
        list2 = list2[::step][:len(list1)]

    result = [x - y for x, y in zip(list1, list2)]
    
    return result

#function to calculate the average euclidean error over the entire trial
def euclidean_error(error_x, error_y):

    euclidean_errors = []  # List to store the Euclidean errors

    for x, y in zip(error_x, error_y):
        euclidean_errors.append(math.sqrt(x**2 + y**2))

    average_error = sum(euclidean_errors) / len(euclidean_errors)
    return average_error

#function to convert csv file to numeric and remove quotes
def remove_quotes_and_convert_to_numeric(data_list):
    return [float(element.strip("'")) for element in data_list]


# def error_along_path(error1, error2, error3, error4, error5):
#       # Calculate the average
#     std1 = np.nanstd([error1, error2, error3,error4,error5])
#     #add std deviation as error bars
#     X1 = [2.855316, 3.854826,4.919766, 6.054046,7.051806]
#     left_full = [error1, error2, error3, error4, error5]

#     # left_full_yer = [std1,std2,std3,std4,std5,std6]

#     plt.errorbar(X1, left_full, fmt='-D', yerr = std1, label ='average error')
#     plt.xlabel('Longitudinal position along path [m]')
#     plt.ylabel('Average Euclidean Error [m]')
#     plt.legend(loc='upper left', fontsize =12)
#     plt.grid(linewidth=0.4)
#     plt.ylim([0,1.2])
#     plt.savefig(errorplot_path)
#     plt.show()


# Subscribers
cov = rospy.Subscriber('covx', cov, callback)
path = rospy.Subscriber('trackedreflectors', filteredReflectors, callback2)
dbscan = rospy.Subscriber('dbscanimage', reflectors, callback4)

radar_image = rospy.Subscriber('radarImage', reflectors, callback3)

# Spin the node until it is stopped
while not rospy.is_shutdown():
  rospy.spin()

#populate the following variables with ground truth data from vicon csv files
gtx1 = read_csv_column(file_path, col1)
gtx2 = read_csv_column(file_path, col2)
gtx3 = read_csv_column(file_path, col3)
gtx4 = read_csv_column(file_path, col4)
gtx5 = read_csv_column(file_path, col5)

#y values:
gty1 = read_csv_column(file_path, col6)
gty2 = read_csv_column(file_path, col7)
gty3 = read_csv_column(file_path, col8)
gty4 = read_csv_column(file_path, col9)
gty5 = read_csv_column(file_path, col10)
# Remove quotes and convert to numeric values
gtx1 = remove_quotes_and_convert_to_numeric(gtx1)
gtx2 = remove_quotes_and_convert_to_numeric(gtx2)
gtx3 = remove_quotes_and_convert_to_numeric(gtx3)
gtx4 = remove_quotes_and_convert_to_numeric(gtx4)
gtx5 = remove_quotes_and_convert_to_numeric(gtx5)

gty1 = remove_quotes_and_convert_to_numeric(gty1)
gty2 = remove_quotes_and_convert_to_numeric(gty2)
gty3 = remove_quotes_and_convert_to_numeric(gty3)
gty4 = remove_quotes_and_convert_to_numeric(gty4)
gty5 = remove_quotes_and_convert_to_numeric(gty5)

#error
error1 = error(x1, gtx1)
error1 = [abs(x) for x in error1]

error1y = error(y1, gty1)
error1y = [abs(x) for x in error1y]

euc1 = euclidean_error(error1, error1y)
print("error1",euc1)


error2 = error(x2, gtx2)
error2 = [abs(x) for x in error2]
error2y = error(y2, gty2)
error2y = [abs(x) for x in error2y]
euc2 = euclidean_error(error2, error2y)
print("error2", euc2)

error3 = error(x3, gtx3)
error3 = [abs(x) for x in error3]
error3y = error(y3, gty3)
error3y = [abs(x) for x in error3y]
euc3 = euclidean_error(error3, error3y)
print("error3", euc3)

error4 = error(x4, gtx4)
error4 = [abs(x) for x in error4]
error4y = error(y4, gty4)
error4y = [abs(x) for x in error4y]
euc4 = euclidean_error(error4, error4y)
print("error4", euc4)

error5 = error(x5, gtx5)
error5 = [abs(x) for x in error5]
error5y = error(y5, gty5)
error5y = [abs(x) for x in error5y]
euc5 = euclidean_error(error5, error5y)
print("error5", euc5)
# print("x err",error5)
# print("y err",error5y)

total_error = (euc1+euc2+euc3+euc4+euc5)/5

x = np.arange(len(data1))

print("average euclidean error", total_error)

plot_covariance(data1, x1, gtx1, error1, data1neg, x, cov1_path)
plot_covariance(data2, x2, gtx2, error2, data2neg, x, cov2_path)
plot_covariance(data3, x3, gtx3, error3, data3neg, x, cov3_path)
plot_covariance(data4, x4, gtx4, error4, data4neg, x, cov4_path)
plot_covariance(data5, x5, gtx5, error5, data5neg, x, cov5_path)
#plot_covariance(data1, y0, gty1, error5, data1neg, x, cov5_path)

#print(cluster_x)

create_scatter_plot(pcl_x,pcl_y, cluster_x, cluster_y, x1,y1, x2,y2,x3,y3,x4,y4,x5,y5,"Y [m]", "X [m]")