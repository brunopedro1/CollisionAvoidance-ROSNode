"""
Instituto Superior Tecnico, University of Lisbon

Part of the Master`s Thesis in Aerospace Engineering:
"Development of a Sense and Avoid System for Small Fixed-wing UAV" 

Author: Bruno Pedro

2024
"""


import rospy
from custom_msgs.msg import RangeYaw
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, Trajectory, VFR_HUD
from std_msgs.msg import Header, Float64
from sensor_msgs.msg import Imu
import numpy as np
import math as m
from collections import deque
from filterpy.kalman import KalmanFilter
import time as t
import threading
import params
from tf.transformations import quaternion_multiply, quaternion_from_euler, euler_from_quaternion
from mavros_msgs.srv import CommandLong
from matplotlib import pyplot as plt
import csv


class Vehicle:
    def __init__(self):
        print("Initializing Vehicle class...")
        
        # Create lock for safe access/writing of variables inside threads
        self.lock = threading.Lock()        
        
        self.local_ENU_pos_x = 0
        self.local_ENU_pos_y = 0
        self.local_ENU_pos_z = 0
        
        self.IMUyaw_ENU_rad = 0
        self.COMPASSyaw_ENU_deg = 0
                
        rospy.wait_for_service("mavros/cmd/command")
        self.command_service = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)    
	    
        self.system_id = 140
        self.component_id = 196
        self.autopilot = 8
        self.base_mode = 0
        self.custom_mode = 0
        self.system_status = 3
        
        # Save results
        self.local_pose_file = 'local_pose35.csv'
        csvLocalFile = open(self.local_pose_file, mode='w', newline='') 
        self.local_pose_writer = csv.writer(csvLocalFile)
        self.local_pose_writer.writerow(['timestamp','local_ENU_pos_x','local_ENU_pos_y','local_ENU_pos_z'])
        
        self.vfr_hud_file = 'vfr_hud35.csv'
        csvVFRHUDFile = open(self.vfr_hud_file, mode='w', newline='') 
        self.vfr_hud_writer = csv.writer(csvVFRHUDFile)
        self.vfr_hud_writer.writerow(['timestamp','airspeed','groundspeed'])
	    
    def vehicle_state_callback(self, state):
        if state.connected:
            rospy.loginfo("state: connected to FCU")
    
    def send_heartbeat(self):
        
        self.state_sub = rospy.Subscriber("mavros/state", State, self.vehicle_state_callback)

        while not rospy.is_shutdown():
            try:
                self.command_service(
                    broadcast=False,
                    command=176,
                    confirmation=0,
                    param1=self.base_mode,
                    param2=self.custom_mode,
                    param3=0, param4=0, param5=0, param6=0, param7=0)    
        	
                rospy.loginfo("sent heartbeat")
        	
            except rospy.ServiceException:
                rospy.loginfo("rip") 
        	
            rospy.sleep(0.1)
        	        	
    def subscribe_vehicle_state(self):
    	# Subscribe to current connection state of PX4
    	self.state_sub = rospy.Subscriber("mavros/state", State, self.vehicle_state_callback)
    
    def subscribe_local_position(self):
        # Subscribe to current local position NED
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        
    def subscribe_vfr_hud(self):
        # Subscribe to current local position NED
        self.vfr_hud_sub = rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self.vfr_hud_callback)
    
    def subscribe_desired_trajectory(self):
        # Subscribe to current desired trajectory
        self.desired_trajectory_sub = rospy.Subscriber("/mavros/trajectory/desired", Trajectory, self.desired_trajectory_callback)
        
    def subscribe_imu_data(self):
        # Subscribe to IMU data
        self.imu_data_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_data_callback)
    
    def subscribe_compass_data(self):
        # Subscribe to IMU data
        self.compass_data_sub = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.compass_data_callback)
    
    def local_pose_callback(self, data):
        self.local_ENU_pos_x = data.pose.position.x
        self.local_ENU_pos_y = data.pose.position.y
        self.local_ENU_pos_z = data.pose.position.z
        print("LOCAL POSITION ENU: x=%.2f y=%.2f z=%.2f" %(self.local_ENU_pos_x, self.local_ENU_pos_y, self.local_ENU_pos_z))
	    
        timestamp = t.time()
        self.local_pose_writer.writerow([timestamp, self.local_ENU_pos_x, self.local_ENU_pos_y, self.local_ENU_pos_z])
            
    def vfr_hud_callback(self, data):
        self.airspeed = data.airspeed
        self.groundspeed = data.groundspeed
        print("airspeed=%.2f  groundspeed==%.2f" %(self.airspeed, self.groundspeed))
	    
        timestamp = t.time()
        self.vfr_hud_writer.writerow([timestamp, self.airspeed, self.groundspeed])
        
    def desired_trajectory_callback(self, data):
        # Current waypoint (unmodified by FlightTaskAutoManager)
        self.current_waypoint_x = data.point_2.position.x
        self.current_waypoint_y = data.point_2.position.y
        self.current_waypoint_z = data.point_2.position.z
        rospy.loginfo("Current desired waypoint: x=%.2f y=%.2f z=%.2f" %(self.current_waypoint_x, self.current_waypoint_y, self.current_waypoint_z))
        
    def imu_data_callback(self, data):
        # Orientation quarternion (ENU frame)
        q_ENU = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

        # Euler angles (ENU frame)
        with self.lock:
            self.IMUroll_ENU_rad, self.IMUpitch_ENU_rad, self.IMUyaw_ENU_rad = euler_from_quaternion(q_ENU)
        
        rospy.loginfo("IMU yaw deg: %.2f" %(m.degrees(self.IMUyaw_ENU_rad)))
        
    def compass_data_callback(self, data):
        with self.lock:
            self.COMPASSyaw_ENU_deg = data.data-45
        

class ObstacleDetector:
    def __init__(self, vehicle):
        print("Initialzing ObstacleDetector class...")
        
        self.vehicle = vehicle
       
        # Create lock for safe access/writing of variables inside threads
        self.lock = threading.Lock()

        # Initialize variables to store range readings and yaw angles
        self.sonar1_range = None
        self.filtered_sonar1_range = None
        self.sonar1_range_t = 0
                
        self.sonar2_range = None
        self.filtered_sonar2_range = None
        self.sonar2_range_t = 0
        
        self.laser1_range = None
        self.filtered_laser1_range = None
        self.laser1_range_t = 0
        
        self.laser2_range = None
        self.filtered_laser2_range = None
        self.laser2_range_t = 0
        
        self.lidar_range = None
        self.filtered_lidar_range = None
        self.lidar_range_t = 0
        self.lidar_yaw = None


        # Initialize variables of polar coordinates of the obstacle position in body frame
        self.sonar1_radial = None
        self.sonar1_azimuth = None
        
        self.sonar2_radial = None
        self.sonar2_azimuth = None
        
        self.laser1_radial = None
        self.laser1_azimuth = None
        
        self.laser2_radial = None
        self.laser2_azimuth = None
        
        self.lidar_radial = None
        self.lidar_azimuth = None
        
                
        # Initialize kalman filters for sensors 
        self.kf_sonar1 = self.init_KF(params.DT_SONAR, params.P_SONAR, params.R_SONAR, params.Q_SONAR)
        self.kf_sonar2 = self.init_KF(params.DT_SONAR, params.P_SONAR, params.R_SONAR, params.Q_SONAR)
        self.kf_laser1 = self.init_KF(params.DT_LASER, params.P_LASER, params.R_LASER, params.Q_LASER)
        self.kf_laser2 = self.init_KF(params.DT_LASER, params.P_LASER, params.R_LASER, params.Q_LASER)
        
        # Initialize deques to store in memory an history of the previous 3 measurements of each sensor
        self.sonar1_range_history = deque(maxlen=3)
        self.sonar2_range_history = deque(maxlen=3)
        self.laser1_range_history = deque(maxlen=3)
        self.laser2_range_history = deque(maxlen=3)
        self.lidar_range_history = deque(maxlen=3)
       
       
        # Save results
        self.sonar1_file = 'sonar1_35.csv'
        csvSonar1File = open(self.sonar1_file, mode='w', newline='') 
        self.sonar1_writer = csv.writer(csvSonar1File)
        self.sonar1_writer.writerow(['timestamp','range'])
        
        self.sonar1_filtered_file = 'sonar1_filtered_35.csv'
        csvSonar1FilteredFile = open(self.sonar1_filtered_file, mode='w', newline='') 
        self.sonar1_filtered_writer = csv.writer(csvSonar1FilteredFile)
        self.sonar1_filtered_writer.writerow(['timestamp','range_filtered'])
        
        self.sonar2_file = 'sonar2_35.csv'
        csvSonar2File = open(self.sonar2_file, mode='w', newline='') 
        self.sonar2_writer = csv.writer(csvSonar2File)
        self.sonar2_writer.writerow(['timestamp','range'])
        
        self.sonar2_filtered_file = 'sonar2_filtered_35.csv'
        csvSonar2FilteredFile = open(self.sonar2_filtered_file, mode='w', newline='') 
        self.sonar2_filtered_writer = csv.writer(csvSonar2FilteredFile)
        self.sonar2_filtered_writer.writerow(['timestamp','range_filtered'])
        
        self.laser1_file = 'laser1_35.csv'
        csvLaser1File = open(self.laser1_file, mode='w', newline='') 
        self.laser1_writer = csv.writer(csvLaser1File)
        self.laser1_writer.writerow(['timestamp','range'])
        
        self.laser1_filtered_file = 'laser1_filtered_35.csv'
        csvLaser1FilteredFile = open(self.laser1_filtered_file, mode='w', newline='') 
        self.laser1_filtered_writer = csv.writer(csvLaser1FilteredFile)
        self.laser1_filtered_writer.writerow(['timestamp','range_filtered'])
        
        self.laser2_file = 'laser2_35.csv'
        csvLaser2File = open(self.laser2_file, mode='w', newline='') 
        self.laser2_writer = csv.writer(csvLaser2File)
        self.laser2_writer.writerow(['timestamp','range'])
        
        self.laser2_filtered_file = 'laser2_filtered_35.csv'
        csvLaser2FilteredFile = open(self.laser2_filtered_file, mode='w', newline='') 
        self.laser2_filtered_writer = csv.writer(csvLaser2FilteredFile)
        self.laser2_filtered_writer.writerow(['timestamp','range_filtered'])
        
        self.lidar_file = 'lidar_35.csv'
        csvLidarFile = open(self.lidar_file, mode='w', newline='') 
        self.lidar_writer = csv.writer(csvLidarFile)
        self.lidar_writer.writerow(['timestamp','range','yaw'])
        
        
        
    def subscribe_distance_sensors(self):
        # Subscribe to the sensor topics
        self.sonar1_sub = rospy.Subscriber("/mavros/distance_sensor/sonar1_pub", RangeYaw, self.sonar1_data_callback)
        self.sonar2_sub = rospy.Subscriber("/mavros/distance_sensor/sonar2_pub", RangeYaw, self.sonar2_data_callback)
        self.laser1_sub = rospy.Subscriber("/mavros/distance_sensor/laser1_pub", RangeYaw, self.laser1_data_callback)
        self.laser2_sub = rospy.Subscriber("/mavros/distance_sensor/laser2_pub", RangeYaw, self.laser2_data_callback)
        self.lidar_sub = rospy.Subscriber("/mavros/distance_sensor/lidar_pub", RangeYaw, self.lidar_data_callback)

    def laser1_data_callback(self, data):
        with self.lock:   
            self.laser1_range = data.range - 1 
            
            # Save time instant of data reception
            self.laser1_range_t = t.time()
            print("Laser 1 range %.2f m" % (self.laser1_range))
            
            if(1 <= self.laser1_range <= 45):
                self.filtered_laser1_range = self.apply_KF(self.kf_laser1, self.laser1_range)              
                self.laser1_filtered_writer.writerow([self.laser1_range_t, self.filtered_laser1_range])
            else:
                self.filtered_laser1_range = None            
            
            # Get the polar coordinates of the obstacle position in body frame
            self.laser1_radial, self.laser1_azimuth = self.transform_to_body_frame(params.POS_X_LASER1, params.POS_Y_LASER1, params.YAW_LASER1, self.filtered_laser1_range)

        self.laser1_writer.writerow([self.laser1_range_t, self.laser1_range])

    def laser2_data_callback(self, data):
        with self.lock:   
            self.laser2_range = data.range
            
            # Save time instant of data reception
            self.laser2_range_t = t.time()
            print("Laser 2 range %.2f m" % (self.laser2_range))
            
            if(1 <= self.laser2_range <= 45):
                self.filtered_laser2_range = self.apply_KF(self.kf_laser2, self.laser2_range)
            else:
                self.filtered_laser2_range = None            
            
            # Get the polar coordinates of the obstacle position in body frame
            self.laser2_radial, self.laser2_azimuth = self.transform_to_body_frame(params.POS_X_LASER2, params.POS_Y_LASER2, params.YAW_LASER2, self.filtered_laser2_range)
        
        self.laser2_filtered_writer.writerow([self.laser2_range_t, self.filtered_laser2_range])
        self.laser2_writer.writerow([self.laser2_range_t, self.laser2_range])
        
    def sonar1_data_callback(self, data):
        with self.lock:   
            self.sonar1_range = data.range
            # Save time instant of data reception
            self.sonar1_range_t = t.time()
            print("Sonar 1 range %.2f m" % (self.sonar1_range))
            
            if(1 <= self.sonar1_range <= 7):
                self.filtered_sonar1_range = self.apply_KF(self.kf_sonar1, self.sonar1_range)
            else:
                self.filtered_sonar1_range = None            
            
            # Get the polar coordinates of the obstacle position in body frame
            self.sonar1_radial, self.sonar1_azimuth = self.transform_to_body_frame(params.POS_X_SONAR1, params.POS_Y_SONAR1, params.YAW_SONAR1, self.filtered_sonar1_range)
        
        self.sonar1_filtered_writer.writerow([self.sonar1_range_t, self.filtered_sonar1_range])
        self.sonar1_writer.writerow([self.sonar1_range_t, self.sonar1_range])

    
    def sonar2_data_callback(self, data):
        with self.lock:   
            self.sonar2_range = data.range
            # Save time instant of data reception
            self.sonar2_range_t = t.time()        
            print("Sonar 2 range %.2f m" % (self.sonar2_range))
            
            if(1 <= self.sonar2_range <= 7):
                self.filtered_sonar2_range = self.apply_KF(self.kf_sonar2, self.sonar2_range)
            else:
                self.filtered_sonar2_range = None            
            
            # Get the polar coordinates of the obstacle position in body frame
            self.sonar2_radial, self.sonar2_azimuth = self.transform_to_body_frame(params.POS_X_SONAR2, params.POS_Y_SONAR2, params.YAW_SONAR2, self.filtered_sonar2_range)
        
        self.sonar2_filtered_writer.writerow([self.sonar2_range_t, self.filtered_sonar2_range])
        self.sonar2_writer.writerow([self.sonar2_range_t, self.sonar2_range])
        
    def lidar_data_callback(self, data):
        with self.lock:
            self.lidar_range = data.range
            self.lidar_yaw = data.current_yaw 
            # Save time instant of data reception
            self.lidar_range_t = t.time()     
            print("Lidar range: %.2f m | yaw: %.2f deg" % (self.lidar_range, self.lidar_yaw))
            
            if(1 <= self.lidar_range <= 45 and -45 <= self.lidar_yaw <= 40):
                self.filtered_lidar_range = self.lidar_range
            else:
                self.filtered_lidar_range = None
                self.lidar_yaw = None

            # Get the polar coordinates of the obstacle position in body frame
            self.lidar_radial, self.lidar_azimuth = self.transform_to_body_frame(params.POS_X_LIDAR, params.POS_Y_LIDAR, self.lidar_yaw, self.filtered_lidar_range)
        
        self.lidar_writer.writerow([self.lidar_range_t, self.filtered_lidar_range, self.lidar_yaw])

    def init_KF(self, dt, P, R, Q):
        kf = KalmanFilter(dim_x=2, dim_z=1)

        # Initial state [position, velocity]
        kf.x = np.array([0., 0.])  

        # State transition matrix (how the state evolves from one step to the next)
        kf.F = np.array([[1., dt],
                         [0., 1.]])

        # Measurement function (how measurements map to the state)
        kf.H = np.array([[1., 0.]])  # We only measure the position (distance)
        
        kf.P = P
        kf.R = R
        kf.Q = Q
        
        return kf

    def apply_KF(self, kf, measurement):
        kf.predict()
        kf.update(np.array([measurement]))

        return kf.x[0]  #position 

    
    def transform_to_body_frame(self, pos_x_sensor, pos_y_sensor, yaw_sensor, range_sensor):
        """
        Given the cartesian position (x,y), orientation (yaw), in body frame, and range measurement of the sensor.
        Get the polar coordinates (radial, azimuth) of the measured point in the UAV body frame.
        
        """
        if range_sensor is not None and yaw_sensor is not None:
            # Radial polar coordinate
            radial = m.sqrt((range_sensor*m.cos(m.radians(yaw_sensor))+pos_x_sensor)**2 + (range_sensor*m.sin(m.radians(yaw_sensor))+pos_y_sensor)**2)
            
            # Azimuth polar coordinate
            azimuth_rad = m.atan((range_sensor*m.sin(m.radians(yaw_sensor))+pos_y_sensor)/(range_sensor*m.cos(m.radians(yaw_sensor))+pos_x_sensor))
            
            azimuth = m.degrees(azimuth_rad)         
        else:
            radial = None
            azimuth = None   

        return radial, azimuth
            


class AvoidancePathGenerator:
    def __init__(self, vehicle, obstacle_detector):
        print("Initializing AvoidancePathGenerator class...")
        
        self.vehicle = vehicle
        self.obstacle_detector = obstacle_detector

        # Create lock for safe access/writing of variables inside threads
        self.lock = threading.Lock()
        
        # Save results
        self.angular_sections_file = 'angular_sections35.csv'
        csvAngularSectionsFile = open(self.angular_sections_file, mode='w', newline='') 
        self.angular_sections_writer = csv.writer(csvAngularSectionsFile)
        
        self.angular_sections_ENU_file = 'angular_sections_ENU35.csv'
        csvAngularSectionsENUFile = open(self.angular_sections_ENU_file, mode='w', newline='') 
        self.angular_sections_ENU_writer = csv.writer(csvAngularSectionsENUFile)
                
        self.polar_histogram_file = 'polar_histogram35.csv'
        csvPolarHistogramFile = open(self.polar_histogram_file, mode='w', newline='') 
        self.polar_histogram_writer = csv.writer(csvPolarHistogramFile)
        self.polar_histogram_writer.writerow(['timestamp','polar_histogram'])
        
        self.polar_histogram_ENU_file = 'polar_histogram_ENU35.csv'
        csvPolarHistogramENUFile = open(self.polar_histogram_ENU_file, mode='w', newline='') 
        self.polar_histogram_ENU_writer = csv.writer(csvPolarHistogramENUFile)
        self.polar_histogram_ENU_writer.writerow(['timestamp','polar_histogram_ENU'])
        
        self.radials_azimuths_file = 'radials_azimuths35.csv'
        csvRadialsAzimuthsFile = open(self.radials_azimuths_file, mode='w', newline='') 
        self.radials_azimuths_writer = csv.writer(csvRadialsAzimuthsFile)
        self.radials_azimuths_writer.writerow(['timestamp','radial_sonar1','radial_sonar2','radial_laser1','radial_laser2','radial_lidar','azimuth_sonar1','azimuth_sonar2','azimuth_laser1','azimuth_laser2','azimuth_lidar'])
        
        self.valid_sections_file = 'valid_setions35.csv'
        csvValidSectionsFile = open(self.valid_sections_file, mode='w', newline='') 
        self.valid_sections_writer = csv.writer(csvValidSectionsFile)
        self.valid_sections_writer.writerow(['timestamp','valid_sections'])
        
        self.valid_sections_ENU_file = 'valid_setions_ENU35.csv'
        csvValidSectionsENUFile = open(self.valid_sections_ENU_file, mode='w', newline='') 
        self.valid_sections_ENU_writer = csv.writer(csvValidSectionsENUFile)
        self.valid_sections_ENU_writer.writerow(['timestamp','valid_sections_ENU'])
        
        self.sections_chosen_file = 'section_chosen35.csv'
        csvSectionChosenFile = open(self.sections_chosen_file, mode='w', newline='') 
        self.section_chosen_writer = csv.writer(csvSectionChosenFile)
        self.section_chosen_writer.writerow(['timestamp','section_chosen'])
        self.section_chosen_instant = 0
        
        self.sections_chosen_ENU_file = 'section_chosen_ENU35.csv'
        csvSectionChosenENUFile = open(self.sections_chosen_ENU_file, mode='w', newline='') 
        self.section_chosen_ENU_writer = csv.writer(csvSectionChosenENUFile)
        self.section_chosen_ENU_writer.writerow(['timestamp','section_chosen_ENU'])
        
                
                
    def polar_histogram(self, min_angle=params.MIN_ANGLE, max_angle=params.MAX_ANGLE, step_angle=params.STEP_ANGLE, gamma=params.GAMMA):
        
        radials = np.zeros(5)
        azimuths = np.zeros(5)
        
        # Define angular sections (bin edges) of the polar histogram
        self.angular_sections = np.arange(min_angle, max_angle+step_angle, step_angle)
        self.angular_sections_ENU = np.arange(0, 360+step_angle, step_angle)
        self.angular_sections_writer.writerow(self.angular_sections.tolist())
        self.angular_sections_ENU_writer.writerow(self.angular_sections_ENU.tolist())
        
        # Initialize histogram array
        self.histogram = np.zeros(len(self.angular_sections))
        self.histogram_ENU = np.zeros(len(self.angular_sections_ENU))
        self.polar_histogram_instants = np.zeros(len(self.angular_sections))
        self.polar_histogram_ENU_instants = np.zeros(len(self.angular_sections_ENU))   
        
        # Create histogram plot
        #self.create_histogram_ENU_plot()
        
        # Auxiliar arrays of obstacle densities
        h_ks = np.zeros(len(self.angular_sections))         
        h_k_neighbourhoods = np.zeros(len(self.angular_sections)) 
        h_ks_ENU = np.zeros(len(self.angular_sections_ENU))         
        h_k_neighbourhoods_ENU = np.zeros(len(self.angular_sections_ENU))          
                
        
        # Loop to update the histogram every 0.05 sec (20hz update freq)
        try:
            while not rospy.is_shutdown():
                print("----------------------------------------")
                print("UPDATE POLAR HISTOGRAM")    
                
                with self.lock:
                    # Filter out outdated data (eg. when a sensor stops working)
                    self.filter_old_data()    
                        
                    # Clean old values of the histogram
                    #self.clean_old_histogram(params.TIME_CLEAN_BINS)
                    self.clean_old_histogram_ENU(params.TIME_CLEAN_BINS)
                                   
                # Update radials and azimuths to the most recent data
                radials = np.array([self.obstacle_detector.sonar1_radial, self.obstacle_detector.sonar2_radial, self.obstacle_detector.laser1_radial, self.obstacle_detector.laser2_radial, self.obstacle_detector.lidar_radial])
                
                azimuths = np.array([self.obstacle_detector.sonar1_azimuth, self.obstacle_detector.sonar2_azimuth, self.obstacle_detector.laser1_azimuth, self.obstacle_detector.laser2_azimuth, self.obstacle_detector.lidar_azimuth])
                
                #print("radials array: %s" % np.array2string(radials))
                #print("azimuths array: %s" % np.array2string(azimuths))
                
                timestamp = t.time()
                self.radials_azimuths_writer.writerow([timestamp] + radials.tolist() + azimuths.tolist())
                
                # Update histogram
                for azimuth, radial in zip(azimuths, radials):

                    if azimuth is not None and radial is not None:
                        #print("Updating histogram...")
                        
                        # ENU azimuth
                        azimuth_ENU = azimuth + self.vehicle.COMPASSyaw_ENU_deg
                        if azimuth_ENU > 360:
                            azimuth_ENU = azimuth_ENU - 360
                        elif azimuth_ENU < 0:
                            azimuth_ENU = azimuth_ENU + 360
                        print("azimuth_ENU: %f" %azimuth_ENU)
                        
                        # Find section index for current azimuth
                        k_section_ENU = np.digitize(azimuth_ENU, self.angular_sections_ENU) - 1
                        
                        # Compute obstacle density value
                        new_hk_ENU = (50-radial)/50
                        
                        if h_ks_ENU[k_section_ENU] < new_hk_ENU:
                            h_ks_ENU[k_section_ENU] = new_hk_ENU
                            self.polar_histogram_ENU_instants[k_section_ENU] = t.time()

                        
                        for k_neighbour_ENU in range(k_section_ENU-gamma, k_section_ENU+gamma): 
                            if 0 <= k_neighbour_ENU < len(self.angular_sections_ENU) and k_neighbour_ENU != k_section_ENU:
                                a = abs(k_section_ENU - k_neighbour_ENU)
                                h_k_neighbourhoods_ENU[k_neighbour_ENU] = h_ks_ENU[k_section_ENU]
                                if h_ks_ENU[k_neighbour_ENU] < h_k_neighbourhoods_ENU[k_neighbour_ENU]:
                                    h_ks_ENU[k_neighbour_ENU] = h_k_neighbourhoods_ENU[k_neighbour_ENU] # only save the neighour obstacle density if it is higher that the density that was previously on that section 
                                    
                                    # Save time instant
                                    self.polar_histogram_ENU_instants[k_neighbour_ENU] = t.time()
                            else:
                                continue
                        
                        with self.lock:
                            # Save updated histogram
                            self.histogram_ENU = h_ks_ENU[:]
                            print("polar histogram ENU: %s" % np.array2string(self.histogram_ENU))
                            
                            timestamp = t.time()
                            self.polar_histogram_ENU_writer.writerow([timestamp] + self.histogram_ENU.tolist())
                    else:
                        continue
                        
                # Update plot polar histogram
                #self.update_plot_histogram_ENU()      

                rospy.sleep(0.05)
        except rospy.ROSInterruptException:
            pass
        

        
    def filter_old_data(self):
        """
        Eliminate data of distance sensors with more than 0.2 sec
        """
        current_time = t.time()
        if((current_time - self.obstacle_detector.sonar1_range_t) > 0.2):
            print("Sonar 1 data is too old!")
            self.obstacle_detector.sonar1_radial = None
        if((current_time - self.obstacle_detector.sonar2_range_t) > 0.2):
            print("Sonar 2 data is too old!")
            self.obstacle_detector.sonar2_radial = None
        if((current_time - self.obstacle_detector.laser1_range_t) > 0.2):
            print("Laser 1 data is too old!")
            self.obstacle_detector.laser1_radial = None
        if((current_time - self.obstacle_detector.laser2_range_t) > 0.2):
            print("Laser 2 data is too old!")
            self.obstacle_detector.laser2_radial = None
        if((current_time - self.obstacle_detector.lidar_range_t) > 0.2):
            print("Lidar data is too old!")
            self.obstacle_detector.lidar_radial = None 
    
    def clean_old_histogram(self, time_clean_bins):
        """
        Clean the data of the histogram with more that 0.2 sec, making a refresh rate of 5Hz 
        """
        current_time = t.time()
        for k in range(len(self.histogram)):
            if (current_time - self.polar_histogram_instants[k]) > 0.2 and self.histogram[k] != 0:
                print("Cleaning section %f of polar histogram." % self.angular_sections[k])
                self.histogram[k] = 0
            else:
                continue
        
    
    def clean_old_histogram_ENU(self, time_clean_bins):

        current_time = t.time()
        for k in range(len(self.histogram_ENU)):
            if (current_time - self.polar_histogram_ENU_instants[k]) > time_clean_bins and self.histogram_ENU[k] != 0:
                print("Cleaning section %f of polar histogram ENU." % self.angular_sections_ENU[k])
                self.histogram_ENU[k] = 0
            else:
                continue
            
    
    def create_histogram_ENU_plot(self):
        self.fig, self.ax = plt.subplots()
        
        self.bars = self.ax.bar(self.angular_sections_ENU[:], self.histogram_ENU, width=0.9, color='blue')
            
        self.ax.set_xlabel('Azimuth')
        self.ax.set_ylabel('Obstacle Density')
        self.ax.set_title('Polar Histogram ENU')
        
        self.ax.set_ylim(0, 1)
        
        # Display plot
        plt.ion()
        plt.show()
        plt.pause(0.01)
    
        
    def update_plot_histogram_ENU(self):
        print("updating plot")
        for bar, new_obstacle_density in zip(self.bars, self.histogram_ENU):
            bar.set_height(new_obstacle_density)
        
        self.fig.canvas.draw_idle()
        plt.pause(0.0001)
         
    def generate_avoidance_setpoints(self, current_setpoint_x_ENU, current_setpoint_y_ENU, desired_yaw_ENU):
        """
        In case of relevant obstacles detected, find the direction (of the polar histogram sections) with no obstacles that is closer to the direction of the last setpoint given in the current operation (Offboard Mode). Generate setpoints in that direction.
        
        """
        # Offboard mode
        setpoint_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        #setpoint_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        
        try:
            while not rospy.is_shutdown():
                print("--------------------------------------")
                print("GENERATE AVOIDANCE SETPOINTS FUNCTION ")
                print("desired_yaw_ENU: %f" % desired_yaw_ENU)
                
                section_chosen_ENU = desired_yaw_ENU
                            
                if 0 <= desired_yaw_ENU <= 360:
                    with self.lock:
                        # Select sections of the histogram with obstacle density below the threshold
                        k_valid_sections_ENU = np.where(self.histogram_ENU < params.THRESHOLD)[0]
                        print("valid sections ENU: %s" % np.array2string(self.angular_sections_ENU[k_valid_sections_ENU]))
                        
                        timestamp = t.time()
                        self.valid_sections_ENU_writer.writerow([timestamp] + self.angular_sections_ENU[k_valid_sections_ENU].tolist())
                        
                    if len(k_valid_sections_ENU) != 0:

                        # Choose the section closer to desired yaw
                        k_section_chosen_ENU = k_valid_sections_ENU[np.argmin(np.abs(self.angular_sections_ENU[k_valid_sections_ENU]-(0.4*desired_yaw_ENU+0.6*section_chosen_ENU)))]

                        section_chosen_ENU = self.angular_sections_ENU[k_section_chosen_ENU]       
                        print("section chosen ENU: %f" % section_chosen_ENU)
                        self.section_chosen_ENU_writer.writerow([timestamp,section_chosen_ENU])
                        
                        #self.section_chosen_to_desired_instant = t.time()
                        
                        # Determine new linear velocity setpoint (vx, vy)
                        new_setpoint_vx_ENU = params.SETPOINT_STEP*m.sin(m.radians(section_chosen_ENU))
                        new_setpoint_vy_ENU = params.SETPOINT_STEP*m.cos(m.radians(section_chosen_ENU))
                        
                        # Determine local position setpoint (x, y)
                        self.new_setpoint_x_ENU = new_setpoint_vx_ENU + self.vehicle.local_ENU_pos_x
                        self.new_setpoint_y_ENU = new_setpoint_vy_ENU + self.vehicle.local_ENU_pos_y
                        
                        print("new setpoints ENU: vx %f | vy %f" % (new_setpoint_vx_ENU, new_setpoint_vy_ENU))
                        print("new setpoints ENU: x %f | y %f" % (self.new_setpoint_x_ENU, self.new_setpoint_y_ENU))
                        print("--------------------------------------")
                        
                        # Publish new setpoint
                        self.publish_setpoints(self.new_setpoint_x_ENU, self.new_setpoint_y_ENU, setpoint_pub)
                        
                    else:
                        print("No valid histogram sections below threshold")
                else:
                    print("current_setpoint_yaw_ENU is outside the possible angles!")             
                
                rospy.sleep(0.1)
        except rospy.ROSInterruptException:
            pass
        
    
    def publish_setpoints(self, setpoint_x, setpoint_y, publisher):
        # Publish setpoints in ENU frame
        
        # Create and fill setpoint_position message to publish
        setpoint_msg = PoseStamped()
        
        setpoint_msg.header = Header()
        setpoint_msg.header.stamp = rospy.Time.now()
        setpoint_msg.header.frame_id = "map"
        
        setpoint_msg.pose.position.x = setpoint_x
        setpoint_msg.pose.position.y = setpoint_y
        setpoint_msg.pose.position.z = 0
        
        
        publisher.publish(setpoint_msg)
        print(f"Published setpoint x={setpoint_x}, y={setpoint_y}")
        
    def publish_setpoints_vel(self, setpoint_x, setpoint_y, publisher):
        
        # Create and fill setpoint_velocity message to publish
        setpoint_vel_msg = TwistStamped()
    
        setpoint_vel_msg.twist.linear.x = setpoint_x
        setpoint_vel_msg.twist.linear.y = setpoint_y
        setpoint_vel_msg.twist.linear.z = 0
        setpoint_vel_msg.twist.angular.x = 0
        setpoint_vel_msg.twist.angular.y = 0
        setpoint_vel_msg.twist.angular.z = 0
        
        publisher.publish(setpoint_vel_msg)
        print(f"Published setpoint_vel vx={setpoint_x}, vy={setpoint_y}")
        
        

