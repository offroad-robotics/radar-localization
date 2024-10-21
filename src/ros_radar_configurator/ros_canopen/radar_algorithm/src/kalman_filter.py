#!/usr/bin/env python
# 
# Multi Object Tracker code which fuses radar measurements from Continental ARS408 
# and wheel encoder measurements form Clearpath Husky Robot
# We simplify the system with a constant velocity motion model
# This code is configured to use Kalman Filter state estimation with GNN data association 
# Created by: Dean Sacoransky - 17drs3@queensu.ca - May 2023

# Outputs: 
# estimated positions (tracks) of multiple objects on 'trackedreflectors' topic
# corresponding uncertainties/variances in the state estimation on the 'covx' and 'covy' topics

# Inputs: object detection and classifications on topic "/dbscanimage" and velocity reading on the "/husky_a1/husky_velocity_controller/odom" topic

#shutdown this node as soon as the rosbag ends for proper result plots

# Importing ROS messages and dependencies
import rospy
from numpy.linalg import inv
from pb_msgs.msg import reflectors
from pb_msgs.msg import filteredReflectors
from pb_msgs.msg import filteredReflector
from pb_msgs.msg import cov
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rospy.numpy_msg import numpy_msg
import numpy as np
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment


#to do: implement a stop trigger: bug at end of rosbag replay


class MultiObjectTracker(object):

    def __init__(self):

        # **** Initialize variables from handlers ****
        self.motion_x = 0
        self.motion_y = 0
        self.last_measurement = np.empty((0,2), dtype=np.float32)

        # Disturbance related variables
        self.prev_q = np.array([0,0,0])
            
        # Define timer for callback function to calculate motion model computation
        # The time step determines the update rate of the Kalman filter. 
        # It should match the rate at which the measurements and system dynamics are available.
        #/husky_a1/husky_velocity_controller/odom published at ~10 hz
        #/dbscanimage published at ~6.7 hz
        #time step should correspond to less frequent sensor - rate = (1/6.7) ~0.15
        # Adjusting the time step appropriately is important to ensure that the filter reacts to changes in the system's state in a timely manner.
        self.T = rospy.get_param('~rate',0.15) #Frquency = (1/rate) Hz
        self.timer = rospy.Timer(rospy.Duration(self.T), self.timer_callback)
        self.G = np.array([self.T,self.T,self.T,self.T,self.T])
        
        #initialize filter
        self.filter_init()

        # Initialize all topic names
        self.subpubdef()

    # Init for Subscriber and Publisher topics
    def subpubdef(self):        
        # Topics that controller is subscribing to
        self.sub_odom = rospy.Subscriber("/husky_a1/husky_velocity_controller/odom", Odometry, self.handle_odom)
        self.sub_radar = rospy.Subscriber("/dbscanimage", reflectors, self.handle_radar)
        # Topics to that controller is publishing to
        self.filtered_state_pub = rospy.Publisher('trackedreflectors', filteredReflectors, queue_size=10)
        self.filtered_cov_pub = rospy.Publisher('covx', cov, queue_size=10)
        self.filtered_cov_y_pub = rospy.Publisher('covy', cov, queue_size=10)
    
    # Initalization for MOT filter
    def filter_init(self):
        rospy.logwarn("Initializing Filter...")
        #initialize empty array for radar measurements in x and y
        self.x_reading = np.empty((0,2), dtype=np.float32)
        self.y_reading = np.empty((0,2), dtype=np.float32)

        #initialize state space - 5x1 state vector
        #The initial state estimate and covariance matrix represent the initial belief about the system's state. 
        # Setting these values properly based on your prior knowledge can help the filter start with a reasonable estimate
        
        #left curve
        #self.initial_y = np.array([-0.3,-0.3,-0.3,0.2,0.5])
        
        #right curve
        #self.initial_y = np.array([-0.4,-0.4,-0.4,-0.8,-1])
        
        #straight road
        self.initial_y = np.array([-0.6,-0.6,-0.6,-0.6,-0.6])

        self.initial_state = np.array([3,4,5,6,7])

        #initialize covariance values (input to priori step) 
        self.initial_cov = 0.4*np.identity(5)
        self.initial_cov_y = 0.4*np.identity(5)

        #initialize process (Q) and measurement (R) covariance matricies 
        self.Q = 0.01*np.identity(5)  
        # Increasing Q means you are not confident in your model accuracy, and you will trust measurements more
        #therefore, filter will be responsive to changes and updates in the measurements
        # while decreasing Q can make the estimated state change more slowly, and trust the predictions more.
        self.R = 0.8*np.identity(5)
        #A smaller value of R indicates higher confidence in the measurements, 
        # leading to a larger influence of the measurements on the estimated state. 
        # larger value of R indicates lower confidence in the measurements, 
        # reducing their impact on the estimated state.

        #initialize identity matrix F 
        self.F = np.identity(5)
    
    #------------------Publisher Functions ------------------------------------
    #function to publish the updated state estimation of 5 objects. Subscribed by the smooth_pcl_visualizer node for rviz
    def publish_updated_state(self,x,y):
        array = np.column_stack((x,y))
        sorted_indices = np.argsort(array[:, 0])
        sorted_array = array[sorted_indices]
        image = filteredReflectors()
        for radar_object in sorted_array:
            temp = filteredReflector()
            temp.long_dist = radar_object[0]
            temp.lat_dist = radar_object[1]
            image.reflectors.append(temp)
        self.filtered_state_pub.publish(image)
    
    #function to publish the covariance matrix - we index the diagonals which represent the variance (uncertainty) of our estimation
    def publish_updated_cov_x(self,cov_array):
        cov_object = cov()
        cov_object.cov1 = cov_array[0,0]
        cov_object.cov2 = cov_array[1,1]
        cov_object.cov3 = cov_array[2,2]
        cov_object.cov4 = cov_array[3,3]
        cov_object.cov5 = cov_array[4,4]
        self.filtered_cov_pub.publish(cov_object)
    
    def publish_updated_cov_y(self, cov_array):
        cov_object = cov()
        cov_object.cov1 = cov_array[0,0]
        cov_object.cov2 = cov_array[1,1]
        cov_object.cov3 = cov_array[2,2]
        cov_object.cov4 = cov_array[3,3]
        cov_object.cov5 = cov_array[4,4]
        self.filtered_cov_y_pub.publish(cov_object)

    # -------------------------- Utility Functions --------------------------------            
    # Function to calculate Mahalanobis Distance between a predicted state (x,y) and a measurement (x,y)
    # To be used as hypothesis in data association
    def mahalanobis_distance(self, x, y, P):
        # Calculate the Mahalanobis distance between two vectors x and y with covariance P
        diff = x - y
        return np.sqrt(np.dot(np.dot(diff.T, np.linalg.inv(P)), diff))

    #Function to calculate data association matrix with mahalanobis distance
    def global_nearest_neighbour(self, z, x, P_x, P_y, gating_threshold):
        # Global Nearest Neighbour data association with Mahalanobis distance
        # z is a list of measurements from dbscan with x values in row 0 and y values in row 1 (from 0x0 to 2xn)
        # x is a list of state estimates (predictions) with x col 0 and y col 1 (2x5)
        # P_x is a covariance matrix for longitudinal estimation (5x5)
        # P_y is a covariance matrix for lateral estimation (5x5)
        # R is measurement noise covariance matrices (5x5)
       
        num_meas = z.shape[1]  #number of rows aka number of measurements
        num_targets = x.shape[1] #number of predictions always 5
        
        #if no measurements are made
        if z.shape[0] == 0:
            rospy.loginfo("-----empty measurement------")
            associations = np.full(5, 100) #populate association matrix with "100" to indicate that there is no valid association
        #if measurements exist
        else:
            D = np.zeros((num_targets, num_meas)) # Initialize the distance matrix
            
            # Loop over each prediction-measurement pair
            for i in range(num_targets):
                for j in range(num_meas):          

                    covar = np.array([[P_x[i][i], 0 ], [0, P_y[i][i]]]) #2x2 matrix [[var(x_i), cov(x_i,y_i], [cov(y_i, x_i), var(y_i)]]
                    meas = z[:,j].reshape((2,)) #reshape into [x_meas y_meas]
                    pred = x[:,i].reshape((2,)) #reshape into [x_pred y_prd]
                    d = self.mahalanobis_distance(meas, pred, covar) #Compute the Mahalanobis distance between the measurement and the mean
                    if d <= gating_threshold:
                        D[i, j] = d #distance matrix of size (5x num_measurements)
                    else:
                        D[i,j] = 10000
            #--------Optimiation step of GNN -----------
            # find association that minimizes cost with hungarian algorithm
            #while ensuring there are no duplicate associations
            associations = np.full(num_targets, 100)
            row_ind, col_ind = linear_sum_assignment(D)
            for i, j in zip(row_ind, col_ind):
                if D[i, j] == 10000:
                    associations[i] = 100
                else:
                    associations[i] = j

            #populate missing associations with 100 to indicate no valid association
            #associaions is a 1x5 indicating the measurement id's association to predictions 1-5
            #ex [1,2,4, 100, 3] means: pred 0 associated with meas 1, pred 1 associated with meas 2,
            #pred 2 associated with meas 4, pred 3 has no association, pred 4 associated with meas 3
        return associations
    # -------------------------- Kalman Filter Functions --------------------------------

    #Function to compute prediction step of Kalman Filter
    #inputs: state from last timestep (5x1), previous_cov (5x5), F(5x5), G(5x1), u - process sensor(1x1), Q - process noise(5x5)
    #returns state and covariance predictions
    def priori(self, previous_state, u, previous_cov):
        state_priori = np.matmul(self.F,previous_state) - self.G*u    #constant velocity motion model
        temp = np.matmul(self.F,previous_cov)
        cov_priori = np.matmul(temp, self.F.transpose()) + self.Q #(5x5)
        return state_priori, cov_priori
    
    #Function to compute correction step  of kalman filer
    #inputs: state_priori (5x1), measurement dbscan output (from 0x0 to 2xnum_meas), R - meas noise (5x5),, cov_priori(5x5)
    def posteriori(self, state_priori, measurement, cov_x, cov_y):
    
        #---------------Data Association-----------------
        #association matrix will indicate which objects in our state space have corresponding measurements
        #and which objects have missing measurements
        #aka data association decides which measurments we use to update our predictions
        associations = self.global_nearest_neighbour(measurement, state_priori, cov_x, cov_y, 1)


        priori_x = state_priori[0]  #x predictions
        priori_y = state_priori[1]  #y predictions

        #initialize updated state, updated covariance, and kalman gain variables
        state_posteriori_x = np.zeros(5)   
        state_posteriori_y = np.zeros(5)

        cov_posteriori_x = np.zeros((5,5))
        cov_posteriori_y = np.zeros((5,5))

        #Kalman gain puts weight on the error b/w measurement and estimate (the innovation)
        #The gain controls how much you trust the measurement over the estimate
        #high gain = relies more on measurement sensor (typically if the R is low)

        kalman_gain_x = np.zeros((5,5))
        kalman_gain_y = np.zeros((5,5))

        #loop through state space (5 objects)
        for j in range(len(state_priori[1])):
            
            #---------Missed Detection Handler----------------
            #if object j has no valid measurement association....
            if associations[j] ==100:
                #print("no association for object ", j)
                #print("populating for missed detection....")
                #print("These predictions are not going through motion model...????")
                
                #updated state estimate is set to predicted state estimate
                state_posteriori_x[j] = priori_x[j]
                state_posteriori_y[j] = priori_y[j]

                #do we update the covariance and kalman gain??
                #pretty sure kalman gain and covariance are the same for x and y of the same object?
                
                #kalman_gain_x[j][j] = cov_x[j][j]/(cov_x[j][j]+self.R[j][j])
                cov_posteriori_x[j][j] = cov_x[j][j] #*kalman_gain_x[j][j]

                #kalman_gain_y[j][j] = cov_y[j][j]/(cov_y[j][j]+self.R[j][j])
                cov_posteriori_y[j][j] = cov_y[j][j] #*kalman_gain_y[j][j]
            
            #-------------Correction Step of KF----------------- 
            #if object j has a valid measurement association
            else:
                #print("associating object", j, "with measurement",associations[j])
                z_xj = measurement[0,associations[j]]
                z_yj = measurement[1,associations[j]]

                kalman_gain_x[j][j] = cov_x[j][j]/(cov_x[j][j]+self.R[j][j])
                kalman_gain_x[np.isnan(kalman_gain_x)] = 0        
                
                #print("kalman gain", kalman_gain)
                state_posteriori_x[j] =priori_x[j] + kalman_gain_x[j,j]*(z_xj - priori_x[j])
                temp = (self.F - kalman_gain_x*self.F)
                cov_posteriori_x[j][j] = temp[j][j]*cov_x[j][j]

                kalman_gain_y[j][j] = cov_y[j][j]/(cov_y[j][j]+self.R[j][j])
                kalman_gain_y[np.isnan(kalman_gain_y)] = 0
                #print("kalman gain", kalman_gain)
                state_posteriori_y[j] =priori_y[j] + kalman_gain_y[j,j]*(z_yj - priori_y[j])
                temp = (self.F - kalman_gain_y*self.F)
                cov_posteriori_y[j][j] = temp[j][j]*cov_y[j][j]

        return state_posteriori_x, cov_posteriori_x, state_posteriori_y, cov_posteriori_y

    # Function to compute one cycle of Kalman filter
    def kalman_filter(self, initial_state, initial_y, initial_cov,initial_cov_y, last_measurement, motion_x, motion_y):
        #predict - two seperate invocations because v_x and v_y are different
        x_predict,cov_x_predict = self.priori(self.initial_state, self.v_x, self.initial_cov)
        y_predict, cov_y_predict = self.priori(self.initial_y, self.v_y, self.initial_cov_y)
        
        prediction = np.array((x_predict, y_predict))
        
        #associate and update
        self.x_update, self.cov_x_update, self.y_update, self.cov_y_update = self.posteriori(prediction, self.last_measurement, cov_x_predict, cov_y_predict)
        
        return self.x_update, self.cov_x_update, self.y_update, self.cov_y_update


    # -------------------------- Sensor Data Handlers --------------------------------
    
    def handle_odom(self, data):
        #read husky velocity information - process sensor
        self.motion_x = data.twist.twist.linear.x #robots forward direction is considered x (longitudinal direction)
        self.motion_y = data.twist.twist.linear.y #lateral direction (should always be zero)
    
    def handle_radar(self, data):
        reflector_list = (data.reflectors)
        length = (len(data.reflectors)) #number of waypoints in of reflector path
        for i in range(0, length):   #accounts for the possibility of dbscan outputting >5 returns
            x = reflector_list[i].long_dist 
            y = reflector_list[i].lat_dist
            temp_x = np.array(x, dtype=np.float32)
            self.x_reading = np.append(self.x_reading,temp_x)
            temp_y = np.array(y, dtype=np.float32)
            self.y_reading = np.append(self.y_reading,temp_y)
            temp_x=np.array([])
            temp_y = np.array([])

        last_measurement_x = self.x_reading
        last_measurement_y = self.y_reading
        self.last_measurement = np.array((last_measurement_x, last_measurement_y)) # 2xn array, where n is number of measured objects
        #row 0 represents x values and row 1 represents y values. Column 0 represents x,y of object 0
        
        self.x_reading= np.array([])
        self.y_reading = np.array([])

    # Callback for computing Algorithm at defined frequency (1/rate Hz) 
    def timer_callback(self, event):

        # ------------- Start polling data ------------
        # Define sensor data at the beginning of the callback function to be used throughout callback function
        
        # Define velocity of vehicle
        self.v_x = self.motion_x  
        self.v_y = self.motion_y
        
        #define radar measurement aka dbscan output
        self.detector = self.last_measurement
        
        #--------------Compute KALMAN Filter Algorithm one time-------------
        self.x_update, self.cov_x_update, self.y_update, self.cov_y_update = self.kalman_filter(self.initial_state, self.initial_y, self.initial_cov, self.initial_cov_y, self.detector, self.v_x, self.v_y)

        # Prepare state variables for recursion
        self.initial_state = self.x_update
        self.initial_cov = self.cov_x_update
        self.initial_y = self.y_update
        self.initial_cov_y = self.cov_y_update
        # ---------------- Publishing Space ---------------
        #publish updated state and covariance values
        self.publish_updated_state(self.x_update, self.y_update)
        self.publish_updated_cov_x(self.cov_x_update)
        self.publish_updated_cov_y(self.cov_y_update)               
     
    
if __name__ == '__main__':
    # init()
    rospy.init_node('MOT')
    
    # Create the MOT Object
    MultiObjectTracker()

    # Wait for messages on topics, go to callback function when new messages arrive.
    rospy.spin()