#!/usr/bin/env python
# 
# Path Following Controller code for Clearpath Husky Robot (or any differential drive/unicycle robot) 
# This code is configured to use Feedback Linearization, Model Predictive Control with Feedback Linearization or NMPC
# Created by: Michael Fader - 12mthf@queensu.ca - December 2020
# 
# Drives the vehicle along a path received on the "/filteredPath" topic, 
# given a position estimate and velocity on the "/odometry/filtered_map" topic

# ------- Trigger to start code: rostopic pub start_trigger std_msgs/Empty "{}" --once --------

import rospy
import tf
import copy
import numpy as np

# Importing ROS messages from standard ROS messages
from std_msgs.msg import Empty, Bool
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PointStamped, PoseStamped, Twist
from offroad_msgs.msg import ControllerInfo
from sensor_msgs.msg import Joy


class PathFollowingController(object):

    def __init__(self):

        # **** Initialize variables from handlers ****
        self.prev_time = None
        self.path_points = None
        self.est_pose = None
        self.vel = None
        self.c = 0
        self.c_fut = 0
        self.flag = 0

        # Initalize options when using in field testing or simulation
        self.trigger = rospy.get_param('~autotrigger', False)

        # Define button that will be used as a deadman switch for path following
        self.use_estop = rospy.get_param('~use_estop', True)             

        # Set Deadman to button[1] which is the 'A' on the Logitech joysticks
        self.estop = False
        self.estop_button = 1
        
        # **** Get vehicle and controller parameters ****
        # Saturating steering rate to be no more than 2 rad/s (115 deg/sec)
        self.sat_omega_lim = rospy.get_param('~sat_omega_lim', 2)                   
        
        # Defining velocities for the vehicle (Desired and minimum)
        self.v_des = rospy.get_param('~const_vel',0.1)                                # [m/s] Desired vehicle speed
        self.v_min = rospy.get_param('~v_min', 0.5)                                 # [m/s] Vehicle speed used in the control law if the vehicle is moving slower than this
        
        # Define parameter for Path Travelled
        self.path_traveled = Path()
        self.poi = PointStamped()
       
        # Define timer for callback function to calculate control signal computation
        self.T = rospy.get_param('~rate',0.1)
        self.timer = rospy.Timer(rospy.Duration(self.T), self.timer_callback)      # Sampling rate of controller - defined in launch file
        
        # Setup to chose the correct controller based on the launch file
        self.controller_selector = rospy.get_param('~controller_selector','fbl')
        if self.controller_selector == 'fbl':
            # Call init function to set up fbl gains
            self.fbl_init()

        elif self.controller_selector == 'mpcfbl':
            # Call init function to set up MPC+FBL optimization and gains
            self.mpcfbl_init()

        elif self.controller_selector == 'nmpc':
            # Call init function to set up NMPC optimization and gains
            self.nmpc_init()
        else:
            print('Select proper controller')

        # Initialize all topic names
        self.subpubdef()

    # Init for Subscriber and Publisher topics
    def subpubdef(self):

        # Parameterize topics
        self.cmdvel_topic = rospy.get_param('~cmdvel', 'husky_a1/cmd_vel')
        self.estop_topic = rospy.get_param('~husky_estop', 'husky_a1/e_stop')
        self.joy_topic = rospy.get_param('joy', 'husky_a1/joy')
        self.estimate_topic = rospy.get_param('~estimate', '/husky_a1/odometry/filtered')
        self.path_topic = rospy.get_param('~desired_path', 'filteredPath')
        self.starter_topic = rospy.get_param('~start_trigger', 'start_trigger')
        self.stop_topic = rospy.get_param('~stop_trigger', 'stop_trigger')
        
        # Topics to that controller is publishing to
        self.pub_end = rospy.Publisher(self.stop_topic, Empty, queue_size=1)                        # Publish to topic when at the end of the path
        self.pub_estop = rospy.Publisher(self.estop_topic, Bool, queue_size=1)                      # Publish to estop topic to put husky in soft estop mode
        self.pub_info = rospy.Publisher('~controller_info', ControllerInfo, queue_size=1)           # Publish the info from the controller
        self.pub_pathtraveled = rospy.Publisher('~path_travelled', Path, queue_size=1)              # Publish path message of where vehicle has travelled
        self.pub_twist = rospy.Publisher(self.cmdvel_topic, Twist, queue_size=100)                  # Publish to command velocities (v, omega) to the vel controller
        self.pub_poi = rospy.Publisher('~closest_point', PointStamped, queue_size=1)
        self.pub_desiredpath = rospy.Publisher('~des_states', Path, queue_size=1)  
        self.pub_mpcpath = rospy.Publisher('~mpcpredict', Path, queue_size=1)
        self.pub_path_received = rospy.Publisher('~path_received', Empty, queue_size=1)
        self.pub_started_following_path = rospy.Publisher('~following_path', Empty, queue_size=1)

        # Topics that controller is subscribing to
        self.sub_est = rospy.Subscriber(self.estimate_topic, Odometry, self.handle_estimate, queue_size=1)      # To get estimate pose and velocity of vehicle
        self.sub_joy = rospy.Subscriber(self.joy_topic, Joy, self.handle_joy, queue_size=1)                         # To joystick information for deadman pendant
        self.sub_path = rospy.Subscriber(self.path_topic, Path, self.handle_path, queue_size=1)                     # To get path to follow
        self.sub_trig = rospy.Subscriber(self.starter_topic, Empty, self.handle_trigger, queue_size=1)              # Subscribe to empty message to trigger the start of recording

    # Initalization for Feedback Linearization Controller 
    def fbl_init(self):

        # Lookahead Parameters
        self.enable_lookahead = rospy.get_param('~enable_lookahead', False)         # [bool] Set true to get heading from a path point ahead of the vehicle
        self.path_point_spacing = rospy.get_param('~path_point_spacing',0.05)       # [m] Spacing between path points
        self.t_lookahead = rospy.get_param('~t_lookahead', 0.5)                     # [s] Amount of time to look ahead of vehicle when lookahead is enabled
        
        # Controller Gain Parameters and Calculations
        omegaCL = rospy.get_param('~omega_cl',0.7)          # Closed-loop bandwidth for FBL controller
        zeta = rospy.get_param('~zeta',1.5)                 # Damping coeffient
        self.kp = -omegaCL**2                               # Proportional gain
        self.kd = -2*omegaCL*zeta                           # Differential gain

    # Initalization for Model Predictive Control + Feedback Linearization Controller
    def mpcfbl_init(self):
        rospy.logwarn("Full State MPC")

        # Disabling lookahead function that is integrated into the find error function
        self.enable_lookahead = False

        # Define the prediction horizon
        self.p = rospy.get_param('~lookahead',35)
        
        # Define empty arrays for control input - should all be arrays sized px1 (since we are only controlling steering over horizon)
        self.prev_u = np.zeros((self.p, 1))
        self.new_u = np.zeros((self.p, 1))
        self.prev_zk = np.zeros((2, 1))

        # Define the gains and gain matricies
        kr = rospy.get_param('~k_R_fs',1)
        kq1 = rospy.get_param('~k_Q_fs1',3)
        kq2 = rospy.get_param('~k_Q_fs2',3)
        
        # This is related the input and is size pxp
        self.R = kr*np.identity(self.p)              

        # Error gain matrix, size 2px2p
        smallQ = np.array([[kq1, 0.0],[0.0, kq2]])
        Q = 1.0*np.kron(np.identity(self.p),smallQ)

        # Generate matrices for optimization function
        F = np.array([[1.0, self.T],[0, 1.0]])
        G = np.array([[-self.T**2/2 + self.T**2],[self.T]])     # Absolute version of G
        # G = np.array([[0],[self.T]])                          # Approximation of G
        
        self.L = np.zeros((2*self.p,2))
        # Populate L matrix
        for i in range(0, self.p):
            self.L[2*i:2*i+2, 0:2] = np.power(F, i+1)
        
        M = np.zeros((2*self.p,self.p))
        # Populate M matrix
        for i in range(0, self.p):
            for j in range(0, self.p-i):
                # Calculate F raised to an exponent
                F_pow = np.linalg.matrix_power(F,self.p-1-i-j)
                # Calculate the matrix multiplication of F^pow and G
                FmultG = np.matmul(F_pow,G)

                #Take components of F^pow*G and populate a M matrix that is 2pxp in size
                M[2*self.p-1-2*i-1,j] = FmultG[0]
                M[2*self.p-1-2*i,j] = FmultG[1]

        # Make (M^T)*Q, R and K^-1 as class variables
        self.MtransQ = np.matmul(np.transpose(M),Q)
        self.k_inv = np.linalg.inv(np.matmul(self.MtransQ, M) + self.R)

    # Initalization for Nonlinease Model Predictive Control Controller
    def nmpc_init(self):

        rospy.logwarn("NMPC Selected")

        # Disabling lookahead function that is integrated into the find error function
        self.enable_lookahead = False

        # Class parameters for predictive guidance system to lookahead and behind
        self.n_a = 10
        self.n_b = 5

        # Define the prediction horizon and state dimensions
        self.p = rospy.get_param('~lookahead',30)
        self.n = 3          # Dimension of vehicle state is (x,y,theta)
        self.m = 2          # Dimension of inputs to vehicle is (v, omega)
        
        # Define empty arrays for vehicle input commands and updates
        self.u = np.zeros((self.m*self.p, 1))
        self.delta_u = np.zeros((self.m*self.p, 1))

        # Populate the velocity instances with the desired velocity
        for i in range(0,self.p):
            self.u[self.m*i,:] = self.v_des

        # Define the gains and gain matricies
        kr = rospy.get_param('~k_R_nmpc',1)
        kq = rospy.get_param('~k_Q_nmpc',5)

        # Build gain matrices
        self.Q = kq*np.identity(self.n*self.p)
        self.R = kr*np.identity(self.m*self.p)
        
        # Define parameters for optimization
        self.max_iter = rospy.get_param('~max_iter',10)         # This is the max iterations for the optimization per time step
        self.thresh = rospy.get_param('~thresh',0.001)          # This is the threshold on when to stop optimizing

        # Define sizes of Hu and Hz
        self.Hz = np.zeros((self.n*self.p, self.n*self.p))
        self.Hu = np.zeros((self.n*self.p, self.m*self.p))
    
    # -------------------------- Utility Functions --------------------------------            
    # Function to calculate yaw angle from quaternion in Pose message
    def headingcalc(self, pose):
        # Build quaternion from orientation
        quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
        # Find all Euler angles from quaternion transformation
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]      # Extract yaw angle from Euler angles in radians
        
        return yaw

    # Function to saturate steering rate (omega)
    def omega_sat(self, omega):
        # Saturating steering rate when not at steering limits
        if abs(omega) > self.sat_omega_lim:
            omega_fix = self.sat_omega_lim*np.sign(omega)
        else: 
            omega_fix = omega
            
        return omega_fix

    # This function is to build an array of the unicycle vehicle state
    def state(self, loc):
        # The state is [x, y, theta] in [m, m, rad]
        curr_x = loc.pose.position.x
        curr_y = loc.pose.position.y
        curr_theta = self.headingcalc(loc.pose)
        q = np.array([curr_x, curr_y, curr_theta])

        return q
    
    # Unicycle kinematic model for a unicycle (or differential drive) vehicle with respect to center point of the robot
    def unicycle_kinematics(self, u, q):
        # q = (x, y, theta), u = (v, omega)
        new_x = q[0] + self.T*u[0]*np.cos(q[2])
        new_y = q[1] + self.T*u[0]*np.sin(q[2])
        new_theta = q[2] + self.T*u[1]
        new_q = np.array([new_x, new_y, new_theta])

        return new_q

    # Function to find lateral and heading errors with respect to the closest point on path to vehicle (Slow)
    def current_path_errors_slow(self, q):

        # Smart path planner - only look at path points from the previous closest point ahead
        # Array declaration for closest_point_index function      
        N_pp = len(self.path_points.poses)
        x_dist = np.zeros(N_pp)
        y_dist = np.zeros(N_pp)
        self.dist2points = np.zeros(N_pp)

        # Go through path points from previous closest point and find the new closest point to vehicle
        for j in range (0, N_pp):
            x_dist[j] = np.array((q[0] - self.path_points.poses[j].pose.position.x))
            y_dist[j] = np.array((q[1] - self.path_points.poses[j].pose.position.y))
            self.dist2points[j] = np.array(np.sqrt(x_dist[j]**2 + y_dist[j]**2))
        # Find the smallest non-zero distance to a point, and find its index, then add index of previous closest point to make sense in path point array
        self.c_slow = np.argmin(self.dist2points[np.nonzero(self.dist2points)])

        # Local variable to represent closest point at current instance
        targetpoint = self.path_points.poses[self.c_slow]   

        # Heading Error Calculations
        if self.enable_lookahead == True:
            lookahead_index = self.c_slow + int(self.t_lookahead * self.v_des / self.path_point_spacing)
            if lookahead_index >= (N_pp - 1):
                lookahead_index = (N_pp - 1)
            targetpoint_lookahead = self.path_points.poses[lookahead_index]
            targetpoint_heading = self.headingcalc(targetpoint_lookahead.pose)
        else:
            targetpoint_heading = self.headingcalc(targetpoint.pose)      # Heading of the closest point at index c in radians
        
        # Heading error in radians
        eh = q[2] - targetpoint_heading
        # Correct for angle wrapping
        if eh > np.pi:
            eh = eh - 2*np.pi
        if eh < -np.pi:
            eh = eh + 2*np.pi                                  

        # Lateral Error Calculation - considering x and y in N-E-S-W global frame to calculate lateral error    
        del_x = q[0] - targetpoint.pose.position.x       
        del_y = q[1] - targetpoint.pose.position.y
        el = -del_x*np.sin(targetpoint_heading) + del_y*np.cos(targetpoint_heading)
        
        # Summary errors into an array for cleanliness
        pf_errors = np.array([el, eh])

        return pf_errors

      # Function to find lateral and heading errors with respect to the closest point on path to vehicle
    
    # Function to find lateral and heading errors with respect to the closest point on path to vehicle for MPC+FBL (Slow)
    def future_path_errors_slow(self, q):
        
        # Smart path planner- only look at path points from the closest point ahead
        # Array declaration for closest_point_index function      
        N_pp = len(self.path_points.poses)
        x_dist = np.zeros(N_pp)
        y_dist = np.zeros(N_pp)
        self.dist2points = np.zeros(N_pp)

        # Go through all path points and find the closest point to vehicle
        for j in range (0, N_pp):
            x_dist[j] = np.array((q[0] - self.path_points.poses[j].pose.position.x))
            y_dist[j] = np.array((q[1] - self.path_points.poses[j].pose.position.y))
            self.dist2points[j] = np.array(np.sqrt(x_dist[j]**2 + y_dist[j]**2))
        
        self.c_fut_slow = np.argmin(self.dist2points[np.nonzero(self.dist2points)])

        # Local variable to represent closest point at current instance
        targetpoint = self.path_points.poses[self.c_fut_slow] 
        
        # Heading Error Calculations
        targetpoint_heading = self.headingcalc(targetpoint.pose)      # Heading of the closest point at index c in radians
        
        # Heading error in radians
        eh = q[2] - targetpoint_heading
        # Correct for angle wrapping
        if eh > np.pi:
            eh = eh - 2*np.pi
        if eh < -np.pi:
            eh = eh + 2*np.pi                                  

        # Lateral Error Calculation - considering x and y in N-E-S-W global frame to calculate lateral error    
        del_x = q[0] - targetpoint.pose.position.x       
        del_y = q[1] - targetpoint.pose.position.y
        el = -del_x*np.sin(targetpoint_heading) + del_y*np.cos(targetpoint_heading)
        
        # Summary errors into an array for cleanliness
        pf_errors = np.array([el, eh])

        return pf_errors

    # Function to find lateral and heading errors with respect to the closest point on path to vehicle
    def current_path_errors(self, q):

        # Smart path planner - only look at path points from the previous closest point ahead
        # Array declaration for closest_point_index function      
        # N_pp = len(self.path_points.poses)

        # Define how many path points we are looking ahead and behind the current closest point.
        n_a = 10
        n_b = 0
        N_pp = n_a + n_b + 1
        total_pp = len(self.path_points.poses)
        x_dist = np.zeros(N_pp)
        y_dist = np.zeros(N_pp)
        self.dist2points = np.zeros(N_pp)

        # Go through path points from previous closest point and find the new closest point to vehicle
        for j in range (0, N_pp):
            
            # Additive term to loop counter to count through path points from n_b to n_a
            k = self.c - n_b
            # Check that counter and additive term aren't greater than number of path points
            if k+j >= len(self.path_points.poses) - 1:
                k = len(self.path_points.poses) - 1 - j

            x_dist[j] = np.array((q[0] - self.path_points.poses[j+k].pose.position.x))
            y_dist[j] = np.array((q[1] - self.path_points.poses[j+k].pose.position.y))
            self.dist2points[j] = np.array(np.sqrt(x_dist[j]**2 + y_dist[j]**2))
        # Find the smallest distance to a point, and find its index, then add index of previous closest point to make sense in path point array
        self.c = np.argmin(self.dist2points) + (self.c - n_b)

        # Reset self.cfut to be the current closest path point for each iteration
        self.c_fut = self.c

        # Local variable to represent closest point at current instance
        targetpoint = self.path_points.poses[self.c]   

        # Heading Error Calculations
        if self.enable_lookahead == True:
            lookahead_index = self.c + int(self.t_lookahead * self.v_des / self.path_point_spacing)
            if lookahead_index >= (total_pp - 1):
                lookahead_index = (total_pp - 1)
            targetpoint_lookahead = self.path_points.poses[lookahead_index]
            targetpoint_heading = self.headingcalc(targetpoint_lookahead.pose)
        else:
            targetpoint_heading = self.headingcalc(targetpoint.pose)      # Heading of the closest point at index c in radians
        
        # Heading error in radians
        eh = q[2] - targetpoint_heading
        # Correct for angle wrapping
        if eh > np.pi:
            eh = eh - 2*np.pi
        if eh < -np.pi:
            eh = eh + 2*np.pi

        # Lateral Error Calculation - considering x and y in N-E-S-W global frame to calculate lateral error    
        del_x = q[0] - targetpoint.pose.position.x       
        del_y = q[1] - targetpoint.pose.position.y
        el = -del_x*np.sin(targetpoint_heading) + del_y*np.cos(targetpoint_heading)
        
        # Summary errors into an array for cleanliness
        pf_errors = np.array([el, eh])

        return pf_errors

    # Function to find lateral and heading errors with respect to the closest point on path to vehicle for MPC+FBL
    def future_path_errors(self, q):
    
        # Define how many path points we are looking ahead and behind the current closest point.
        n_a = 10
        n_b = 0
        N_pp = n_a + n_b #+ 1
        x_dist = np.zeros(N_pp)
        y_dist = np.zeros(N_pp)
        fut_dist2points = np.zeros(N_pp)

        # Go through path points from previous closest point and find the new closest point to vehicle
        for j in range (0, N_pp):
            
            # Additive term to loop counter to count through path points from n_b to n_a
            k = self.c_fut - n_b
            # Check that counter and additive term aren't greater than number of path points
            if k+j >= len(self.path_points.poses) - 1:
                k = len(self.path_points.poses) - 1 - j

            x_dist[j] = np.array((q[0] - self.path_points.poses[j+k].pose.position.x))
            y_dist[j] = np.array((q[1] - self.path_points.poses[j+k].pose.position.y))
            fut_dist2points[j] = np.array(np.sqrt(x_dist[j]**2 + y_dist[j]**2))
        # Find the smallest distance to a point, and find its index, then add index of previous closest point to make sense in path point array
        self.c_fut = np.argmin(fut_dist2points) + (self.c_fut - n_b)

        # Local variable to represent closest point at current instance
        targetpoint = self.path_points.poses[self.c_fut] 
        
        # Heading Error Calculations
        targetpoint_heading = self.headingcalc(targetpoint.pose)      # Heading of the closest point at index c in radians
        
        # Heading error in radians
        eh = q[2] - targetpoint_heading
        # Correct for angle wrapping
        if eh > np.pi:
            eh = eh - 2*np.pi
        if eh < -np.pi:
            eh = eh + 2*np.pi                                 

        # Lateral Error Calculation - considering x and y in N-E-S-W global frame to calculate lateral error    
        del_x = q[0] - targetpoint.pose.position.x       
        del_y = q[1] - targetpoint.pose.position.y
        el = -del_x*np.sin(targetpoint_heading) + del_y*np.cos(targetpoint_heading)
        
        # Summary errors into an array for cleanliness
        pf_errors = np.array([el, eh])

        return pf_errors

    # -------------------------- Path Following Controller Functions --------------------------------
    # Function to calculate steering rate using Feedback Linearization Controller 
    def fbl_controller(self, q, e):
    
        # Linearized control input using [z1, z2] where e = [el, eh]
        eta = self.kp*e[0] + self.kd*(self.v*np.sin(e[1]))     
      
        # Calculate steering rate based on vehicle errors
        omega = eta/(self.v*np.cos(e[1]))
             
        # Correct / Saturate steering rate if needed
        omega_corr = self.omega_sat(omega)

        return omega_corr
    
    # Function to calculate steering rate using MPC+FBL (Single State or Full State) 
    def mpcfbl_controller(self, q, e):
        
        # Make path message for each MPC loop, so we can see where the sequence wants the vehicle to go.
        self.mpc_predictstates = Path()

        # Defining arrays at the beginning of the function call
        # Define future state array and first future state
        qfut = np.zeros((3, self.p))
        qfut[:,0] = q                # Defining that the first predicted state is the current state

        # Define future error array and first future error
        efut = np.zeros((2, self.p))
        efut[:,0] = e                # Defining that the first predicted error is the current error

        # Define matrix to store predicted FBL states
        self.y = np.zeros((2*self.p, 1))
        self.y[0, 0] = e[0]                         # Storing first fbl state (current lateral error)
        self.y[1, 0] = self.v_des*np.sin(e[1])      # Storing second fbl state (current v*sin(heading error))

        # Define and update change in FBL state
        self.delta_zk = np.zeros((2, 1))
        self.delta_zk[0] = self.y[0,0] - self.prev_zk[0]
        self.delta_zk[1] = self.y[1,0] - self.prev_zk[1]

        # Reset previous FBL states as current states for next time step.
        self.prev_zk[:,0] = self.y[0:2,0]
        
        # Model Predictive Loop
        for i in range(0, self.p-1):

            # Calculate steering rate in the future for DDR
            omegafut = self.prev_u[i]/(self.v_des*np.cos(efut[1,i]))

            # Simulate the vehicle kinematic model for one time step (T)
            qfut[:,i+1] = self.unicycle_kinematics(np.array([self.v_des, omegafut]), qfut[:,i])
            
            # Calculate errors of the future vehicle
            efut[:,i+1] = self.future_path_errors(qfut[:,i+1])
            
            # Populate FBL state storing array for either fullstate or not
            self.y[2*i+2, 0] = efut[0, i+1]                         # Storing first fbl state (lateral error)
            self.y[2*i+3, 0] = self.v_des*np.sin(efut[1, i+1])      # Storing second fbl state (v*sin(heading error))

            # Append to despath path traveled and broadcast path, for visualization
            self.mpc_predictstates.header.frame_id = self.estimate.header.frame_id
            path_pose = PoseStamped()
            path_pose.header.frame_id = self.estimate.header.frame_id
            path_pose.pose.position.x = qfut[0,i+1]
            path_pose.pose.position.y = qfut[1,i+1]

            # Build orientation in quaternion form
            quat_heading = tf.transformations.quaternion_from_euler(0, 0, qfut[2,i+1])
            path_pose.pose.orientation.x = quat_heading[0]
            path_pose.pose.orientation.y = quat_heading[1]
            path_pose.pose.orientation.z = quat_heading[2]
            path_pose.pose.orientation.w = quat_heading[3]
            # Append path
            self.mpc_predictstates.poses.append(copy.deepcopy(path_pose))
        
        # Publish MPC prediction
        self.pub_mpcpath.publish(self.mpc_predictstates)

        # **** Pick optimization for MPC+FBL ****
        # Optimization method 1 - Cost function weighing future states and change in control sequence (delta u_k)
        temp = np.matmul(self.MtransQ, self.y + np.matmul(self.L, self.delta_zk))

        # Optimization method 2 - Cost function weighing future states and magnitude of control sequence (u_k)
        # temp = np.matmul(self.MtransQ, self.y + np.matmul(self.L, self.delta_zk)) + np.matmul(self.R, self.prev_u) 

        # Calculate optimal control update and new control sequence (new_u)
        delta_u = -np.matmul(self.k_inv,temp)
        self.new_u = delta_u + self.prev_u
        
        # Compute steering rate with optimal control input - Based on actual vehicle speed and errors
        omega = self.new_u[0]/(self.v*np.cos(e[1]))

        # Correct / Saturate omega if needed
        omega_corr = self.omega_sat(omega)

        # Redefine previous control output for next iteration
        self.prev_u = self.new_u

        return omega_corr

    # Function to calculate steering rate using NMPC optmization - Based on Ostafew et al. (2014)
    def nmpc_controller(self, q):

        # Build empty arrays for desired states for p time steps ahead
        q_des = np.zeros((3, self.p))
        z_des = np.zeros((self.n*self.p, 1))
        
        # Define ROS path message to view the desired vehicle path (Will run slow if publishing messages)
        # self.nmpc_despathpoints = Path()

        # Simulate vehicle using previous sequence, calculate closest path point to predicted vehicle for desired states
        for i in range (0, self.p):
            # Simulate vehicle one time step forward
            if i == 0:
                q_des[:,0] = self.unicycle_kinematics(np.array([self.v_des, self.u[1,0]]), q)
            else:
                # Simulate the vehicle with the previous control input
                q_des[:,i] = self.unicycle_kinematics(np.array([self.v_des, self.u[self.m*i+1,0]]), q_des[:,i-1])

            # Define arrays based on how many path points we are looking ahead and behind the current closest future predicted point.
            N_pp = self.n_a + self.n_b + 1
            x_dist = np.zeros(N_pp)
            y_dist = np.zeros(N_pp)
            fut_dist2points = np.zeros(N_pp)

            # Additive term to loop counter (j) to count through path points from n_b forward to n_a
            k = self.c_fut - self.n_b

            # Go through path points around previous closest point and find the new closest point to vehicle
            for j in range (0, N_pp):
                
                # Check that counter and additive term aren't greater than number of path points (dont want to look past the last path point)
                if k+j >= len(self.path_points.poses) - 1:
                    k = len(self.path_points.poses) - 1 - j

                x_dist[j] = np.array((q_des[0,i] - self.path_points.poses[j+k].pose.position.x))
                y_dist[j] = np.array((q_des[1,i] - self.path_points.poses[j+k].pose.position.y))
                fut_dist2points[j] = np.array(np.sqrt(x_dist[j]**2 + y_dist[j]**2))

            # Find the smallest distance to all points within observation bubble point, and find its index, then add k to make sense in from previous closest point
            self.c_fut = np.argmin(fut_dist2points) + k

            # Populate z_des matrix with pose of the closest point to our simulated vehicle new point of interest info
            z_des[3*i] = self.path_points.poses[self.c_fut].pose.position.x
            z_des[3*i+1] = self.path_points.poses[self.c_fut].pose.position.y
            z_des[3*i+2] = self.headingcalc(self.path_points.poses[self.c_fut].pose)

            # Append to despath path traveled and broadcast path, for visualization (Warning: NMPC will run slow)
            # self.nmpc_despathpoints.header.frame_id = self.estimate.header.frame_id
            # path_pose = PoseStamped()
            # path_pose.header.frame_id = self.estimate.header.frame_id
            # path_pose.pose = self.path_points.poses[new_poi].pose
            # self.nmpc_despathpoints.poses.append(copy.deepcopy(path_pose))

        # self.pub_desiredpath.publish(self.nmpc_despathpoints)
   
        # Define state and desired state vectors
        qfut = np.zeros((3, self.p))
        z_bar = np.zeros((self.n*self.p, 1))  

        #-------- Optimization --------#
        # occurs within a limited number of iterations
        for i in range (0, self.max_iter):  
            
            # Define ROS path message to view predicted vehicle path
            self.mpc_predictstates = Path()

            # Populate the first predicted state as qfut and zbar
            qfut[:,0] = self.unicycle_kinematics(np.array([self.v_des, self.u[1,0]]), q)
            
            # Populate first three instances of z_bar with first future state
            z_bar[0:3, 0] = qfut[:,0]

            # Populate first instance of Hu at state k
            # Hu_k = self.T*np.array([(np.cos(q[2]), 0), (np.sin(q[2]), 0), (0, 1)])
            Hu_k = np.array([(0, 0), (0, 0), (0, 1) ])  
            self.Hu[0:self.n,0:self.m]= Hu_k

            # Find future vehicle states over prediction horizon         
            for b in range(1, self.p):
                
                # Populate Hu (at q(k+b)) and the rest of Hu (at q(k+b)) 
                # Hu_k = self.T*np.array([(np.cos(qfut[2,b-1]), 0), (np.sin(qfut[2,b-1]), 0), (0, 1) ])    
                # Hu_k = self.T*np.array([(0, 0), (0, 0), (0, 1) ])    

                # Populate Hz (at q(k+b+1)) and the rest of Hu (at q(k+b)) 
                Hz_k = np.array([(1,0,-(self.T*self.v_des*np.sin(qfut[2,b-1]))), (0,1,(self.T*self.v_des*np.cos(qfut[2,b-1]))), (0,0,1)])
                
                self.Hz[self.n*b:self.n*b+self.n,self.n*(b-1):self.n*(b-1)+self.n] = Hz_k
                self.Hu[self.n*b:self.n*b+self.n,self.m*b:self.m*b+self.m]= Hu_k 
                
                # Simulate the vehicle with the previous control input
                qfut[:,b] = self.unicycle_kinematics(np.array([self.v_des, self.u[self.m*b+1,0]]), qfut[:,b-1])

                # Populate z_bar with current state
                z_bar[self.n*b:self.n*b+3, 0] = qfut[:,b]
           
                # Append to despath path traveled and broadcast path, for visualization
                # **** (Warning: NMPC will run slow if publishing ROS msg) ****
                # self.mpc_predictstates.header.frame_id = self.estimate.header.frame_id
                # path_pose = PoseStamped()
                # path_pose.header.frame_id = self.estimate.header.frame_id
                # path_pose.pose.position.x = qfut[0,b]
                # path_pose.pose.position.y = qfut[1,b]

                # # Build orientation in quaternion form
                # quat_heading = tf.transformations.quaternion_from_euler(0, 0, qfut[2,b])
                # path_pose.pose.orientation.x = quat_heading[0]
                # path_pose.pose.orientation.y = quat_heading[1]
                # path_pose.pose.orientation.z = quat_heading[2]
                # path_pose.pose.orientation.w = quat_heading[3]
                # # Append path
                # self.mpc_predictstates.poses.append(copy.deepcopy(path_pose))
            
            # Publish MPC prediction to view in RVIZ
            # self.pub_mpcpath.publish(self.mpc_predictstates)

            # Build H' matrix with Hu and Hz
            H_prime = np.matmul(np.linalg.inv(np.eye(self.n*self.p) - self.Hz),self.Hu)

            # Define difference between desired and actual state
            z_tilde = z_des - z_bar

            # Make corrections for heading difference calculation
            for i in range(0, self.p):
                # Index for heading value in vectors
                k = self.n*i+2

                # Heading error in radians
                # z_tilde[k] = z_des[k] - z_bar[k]     

                # Correct for angle wrapping
                if z_tilde[k] > np.pi:
                    z_tilde[k] = z_tilde[k] - 2*np.pi
                if z_tilde[k] < -np.pi:
                    z_tilde[k] = z_tilde[k] + 2*np.pi




                # # Heading difference in radians
                # if np.sign(z_bar[k]) == np.sign(z_des[k]):
                #     z_tilde[k] = z_des[k] - z_bar[k]     
                # else:
                #     # Condition to properly calculate heading error due to discontinous jump at 180 to -180.
                #     z_tilde[k] = z_des[k] + z_bar[k]     


            # Build derivative of delta_u (Python 2.7 Problems)
            HptransQ = np.matmul(np.transpose(H_prime),self.Q)
            delta_u1 = np.linalg.inv(np.matmul(HptransQ,H_prime) + self.R)
            delta_u2 = np.mat(np.matmul(HptransQ,z_tilde)) - np.matmul(self.R,self.u)
            delta_u = np.matmul(delta_u1, delta_u2)

            flag = 0 # counter to track if delta_u are below thresholds

            # Update u
            self.u = self.u + delta_u

            # Make all velocity inputs equal to the desired velocity
            for i in range(0,self.p):
                self.u[self.m*i,:] = self.v_des

            # Checking to see if all inputs in the sequence are less than minimum input threshold
            # and setting flags if delta_u is < thresh
            for b in range(0, self.p):
                if abs(delta_u[b]) < self.thresh:
                    # if delta_u[b,0] < self.thresh and delta_u[b,0] > -self.thresh:
                    flag+=1
            # print(flag)
            # Break of out optimization if all items in delta_u are small enough
            if flag == self.p:
                print("brokeout")
                break

        # Return the first instance of the new sequence
        omega = self.u[1, 0]

        
        
        return omega

    # -------------------------- Sensor Data Handlers --------------------------------
    # Handler #0 - For topic 
    def handle_trigger(self, data):
        self.trigger = True
        # Notify that path was received
        self.pub_started_following_path.publish(Empty())

    # Handler #1 - For topic /path_recorded
    # Function to handle incoming path data and define as path_points
    def handle_path(self, data):
        # Store path points
        self.path_points = data
        # Notify that path was received
        self.pub_path_received.publish(Empty())
          
    # Handler #2 - For topic /odometry/filetered_map to get pose and velocities
    # Function to handle position estimate of the Husky from robot_localization
    def handle_estimate(self, data):
    
        # Define data from handler as estimate = odometry estimate
        self.estimate = data
        
        # Redefine new variable which is only pose component of Odom msg
        self.est_pose = copy.deepcopy(self.estimate.pose)   
   
        # Redefine new variable which is absolute linear velocity from Odom msg
        self.vel = np.sqrt((self.estimate.twist.twist.linear.x)**2 
                            +(self.estimate.twist.twist.linear.y)**2
                            +(self.estimate.twist.twist.linear.z)**2 )

    # Handler #3 - For topic /joy
    # Function to pay attention to joystick for deadman pendant functionality
    def handle_joy(self, data):
        # Conditional statement to use estop or not (so simulations can use this code)
        if self.use_estop is True:
            # Read estop buttons and if they are on (1), then estop is off       
            if data.buttons[self.estop_button] == 1:
                self.estop= False
                # self.pub_plotter.publish()    # This should start the plotter by publishing a to the starter topic
            else:
                self.estop = True
                rospy.logwarn_throttle(2.5,"Press A button to engage deadman")
            # Publish soft estop param
            self.pub_estop.publish(self.estop)
        else:
            self.estop = False
    
    # Callback for calculating steering commands
    def timer_callback(self, event):
        # print('newloop')

        warning = False
        # Conditional statement to wait until trigger has been recieved - Only relevant when wanting to start controller on remote trigger       
        if self.trigger is not True:
            rospy.logwarn_throttle(2.5, "Controller waiting for start trigger...")
            warning = True

        # Conditional statement to wait until path has been received 
        if self.path_points is None:
            rospy.logwarn_throttle(2.5, "Controller waiting for path...")
            warning = True

        # Conditional statement to wait until vehicle estimate has been received
        if self.est_pose is None:
            rospy.logwarn_throttle(2.5, "Controller waiting for pose...")
            warning = True
        
        if warning:
            return

        # ------------- Start polling data ------------
        # Define sensor data at the beginning of the callback function to be used throughout callback function
        
        # Define velocity of vehicle
        self.v = self.vel   
        
        # Singularity prevention. 
        # If speed is low, omega is nearing infinity. In this case, we just make it self.v_min
        if self.v < self.v_min:
            self.v = self.v_min
        print (self.v)
        

        # Define location of the vehicle within timer function
        loc = self.est_pose

        # Initialize steering and time for beginning of program
        if self.prev_time is None:
            # Initialize timing
            self.prev_time = rospy.Time.now()
            self.seq = 0
        else:
            
            # Establishing the current state of the unicycle vehicle as (x, y, theta)
            q = self.state(loc)

            # Calculate the lateral and heading error of the vehicle where e = [el, eh]
            e = self.current_path_errors(q)
        
            # Timing statement 1 to see controller speed
            tic = rospy.get_time()
            
            # Calculate unicycle steering rate (omega) in rad/s
            if self.controller_selector == 'fbl':
                self.omega = self.fbl_controller(q, e)
            elif self.controller_selector == 'mpcfbl':
                self.omega = self.mpcfbl_controller(q, e)
            elif self.controller_selector == 'nmpc':
                self.omega = self.nmpc_controller(q)
            else:
                rospy.logwarn("No controller selected")
                return

            # Timing statement 2 to see controller speed
            toc = rospy.get_time() - tic
            # print(toc)

            # If we are at the last point on the path is the closest to us 
            # Set command velocity to 0 and omega to 0, set trigger to False.             
            if self.c == (len(self.path_points.poses)-1):
                # v_des = 0.0             
                # self.omega = 0.0
                # self.trigger = False
                # rospy.logwarn_throttle(2.5,"At the end of the path! Stop!")
                
                # self.estop= False        
                # self.pub_estop.publish(self.estop)
                # self.pub_end.publish()
                rospy.logwarn_throttle(2.5,"System wanted to stop")

            else:
                # Constant velocity command
                v_des = self.v_des        

            # ---------------- Publishing Space ---------------

            # Record time now for publishing purposes and add to sequence
            now = rospy.Time.now()
            self.seq = self.seq + 1

            # Build message to publish to topic /controllerinfo
            controller_info = ControllerInfo()
            # Build message header
            controller_info.header.seq = self.seq
            controller_info.header.stamp.secs = int(now.to_sec())
            controller_info.header.stamp.nsecs = now.to_nsec() - int(now.to_sec()) * 1000000000
            controller_info.header.frame_id = self.estimate.header.frame_id
            # Build controller info msg specific info
            controller_info.pathpoint_of_interest = self.c
            controller_info.lateral_error = e[0]
            controller_info.heading_error = e[1]
            controller_info.forward_vel_cmd = v_des
            controller_info.steering_rate_cmd = self.omega
            controller_info.loop_time = toc
            self.pub_info.publish(controller_info)  
            

            # # Build twist message to /cmd_vel
            # cmd_twist = TwistStamped()
            # # Build message header
            # cmd_twist.header.seq = self.seq
            # cmd_twist.header.stamp.secs = int(now.to_sec())
            # cmd_twist.header.stamp.nsecs = now.to_nsec() - int(now.to_sec()) * 1000000000
            # cmd_twist.header.frame_id = self.estimate.child_frame_id        # Setting frame to be child frame of estimate = base_link frame 
            # # Build twist msg specific info
            # cmd_twist.twist.linear.x = v_des
            # cmd_twist.twist.angular.z = self.omega
            # self.pub_twist.publish(cmd_twist)

            # Build twist message to /cmd_vel
            cmd_twist = Twist()
            # Build twist msg specific info
            cmd_twist.linear.x = v_des
            cmd_twist.angular.z = self.omega
            self.pub_twist.publish(cmd_twist)

            # Append to path traveled and broadcast path, for RVIZ visualization and plotting
            self.path_traveled.header.frame_id = self.estimate.header.frame_id
            path_pose = PoseStamped()
            path_pose.header.frame_id = self.estimate.header.frame_id
            path_pose.pose = self.estimate.pose.pose
            self.path_traveled.poses.append(copy.deepcopy(path_pose))
            self.pub_pathtraveled.publish(self.path_traveled)

            # Redefine previous time if you want to use elapsed time - Not used at the moment
            self.prev_time = now            
     
    
if __name__ == '__main__':
    # init()
    rospy.init_node('pathfollow_controller')

    # Create the vehicle path following object    
    PathFollowingController()
    
    # Wait for messages on topics, go to callback function when new messages arrive.
    rospy.spin()
