"""
Instituto Superior Tecnico, University of Lisbon

Part of the Master`s Thesis in Aerospace Engineering:
"Development of a Sense and Avoid System for Small Fixed-wing UAV" 

Author: Bruno Pedro

2024
"""

import rospy
import threading
from collision_avoidance_node import Vehicle
from collision_avoidance_node import ObstacleDetector
from collision_avoidance_node import AvoidancePathGenerator

if __name__ == '__main__':
    
    # Initialize ROS node
    print("Starting collision_avoidance node!")
    rospy.init_node('collision_avoidance', anonymous=True)
    
    # Create instances of the Vehicle, ObstacleDetector, AvoidancePathGenerator classes
    vehicle = Vehicle()
    obstacle_detector = ObstacleDetector(vehicle)
    avoidance_path = AvoidancePathGenerator(vehicle, obstacle_detector)
    
    
    # Creating threads
    #thread_Heartbeat = threading.Thread(target=vehicle.send_heartbeat)
    thread_LocalPosition = threading.Thread(target=vehicle.subscribe_local_position)
    thread_VFRHUD = threading.Thread(target=vehicle.subscribe_vfr_hud)
    #thread_DesiredTrajectory = threading.Thread(target=vehicle.subscribe_desired_trajectory)
    #thread_ImuData = threading.Thread(target=vehicle.subscribe_imu_data)
    thread_CompassData = threading.Thread(target=vehicle.subscribe_compass_data)
    thread_DistanceSensors = threading.Thread(target=obstacle_detector.subscribe_distance_sensors) 
    thread_PolarHistogram = threading.Thread(target=avoidance_path.polar_histogram)
    thread_GenerateSetpoints = threading.Thread(target=avoidance_path.generate_avoidance_setpoints, args=(-10,10,100))
    
    
    # Starting threads
    #thread_Heartbeat.start()
    thread_LocalPosition.start()
    thread_VFRHUD.start()
    #thread_DesiredTrajectory.start()
    #thread_ImuData.start()
    thread_CompassData.start()
    thread_DistanceSensors.start()
    thread_PolarHistogram.start()
    thread_GenerateSetpoints.start()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass