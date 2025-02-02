#!/usr/bin/env python3
import rospy
import os
import json
import math
from std_msgs.msg import String
from utils.msg import localisation
from filterpy.kalman import KalmanFilter
import numpy as np
import threading
import queue
from utils.msg import IMU

class KalmanFilterOptimized:
    def __init__(self, process_noise=1e-5, measurement_noise=1e-2, estimation_error=1.0, initial_values=[0.0, 0.0]):
        self.q = process_noise
        self.r = measurement_noise
        self.p = np.array([estimation_error, estimation_error])
        self.x = np.array(initial_values)

    def update(self, measurements):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (measurements - self.x)
        self.p *= (1 - k)
        return self.x
    def adjust_noise(self, signal_change_rate):
        self.q = max(1e-7, min(1e-5, 1e-6 / (signal_change_rate + 1)))

kf = KalmanFilterOptimized()
kf_posA = 0
kf_posB = 0
raw_posA = 0
raw_posB = 0
data_lock = threading.Lock()
terminate_flag = False

class VehicleController:
    def __init__(self):
        self.publisher_speed_steer = rospy.Publisher('/automobile/command', String, queue_size=1)

    def send_speed_command(self, speed):
        command = {
            'action': '1',  
            'speed': speed
        }
        self.publisher_speed_steer.publish(json.dumps(command)) 
        rospy.loginfo("Sending speed: %f", speed)

    def send_steer_command(self, steer_angle):
        command_steer = {
            'action': '2',  
            'steerAngle': steer_angle
        }
        self.publisher_speed_steer.publish(json.dumps(command_steer)) 

def read_edge_points_from_file(file_name="coordinates.txt"):
    if not os.path.exists(file_name):
        raise FileNotFoundError(f"File {file_name} not found. Ensure it exists.")
    edge_points = []
    with open(file_name, "r") as file:
        lines = file.readlines()
        edge_section = False
        for line in lines:
            if line.strip() == "Edge Points:":
                edge_section = True
                continue
            if edge_section and line.strip():
                x, y = line.strip("()\n").split(", ")
                edge_points.append((float(x), float(y)))
    return edge_points

def convert_angle_to_steer_value(angle):
    steer_value = (angle / 30.0) * 20.5

    if(steer_value > 20.5):
        steer_value = 20.5
    elif(steer_value < -20.5):
        steer_value = -20.5
    return steer_value


data_queue = queue.Queue()



def kalman_thread_func():
    global raw_posA, raw_posB, kf_posA, kf_posB
    prev_raw_data = np.array([0.0, 0.0])

    while not terminate_flag:
        with data_lock:
            raw_data = np.array([raw_posA, raw_posB])
        
        # Calculate signal change rate
        signal_change_rate = np.linalg.norm(raw_data - prev_raw_data)
        kf.adjust_noise(signal_change_rate)
        
        # Apply Kalman Filter
        filtered_data = kf.update(raw_data)
        
        with data_lock:
            kf_posA, kf_posB = filtered_data
        prev_raw_data = raw_data
        rospy.sleep(0.01)

def localisation_callback(data):
    global raw_posA, raw_posB
    with data_lock:
        raw_posA = data.posA
        raw_posB = data.posB

def calculate_distance(posA, posB, targetA, targetB):
    return math.sqrt((posA - targetA) ** 2 + (posB - targetB) ** 2)

yaw_value_new = 0.0

def imu_callback(msg):
    global yaw_value_new 
    yaw_value = msg.yaw 
    yaw_value_new = yaw_value * (180.0 / np.pi) 



def control_vehicle(target_pos):
    global yaw_value_new, kf_posA, kf_posB
    controller = VehicleController()
    speed = 0.00
    steer_value = 0
    targetA, targetB = target_pos 
    rate = rospy.Rate(1) 
    last_curren_angle = 0.0
    rotate_angle_value = 0.0
    right_or_left = True

    while not rospy.is_shutdown():
        filtered_posA = kf_posA
        filtered_posB = kf_posB
        distance = calculate_distance(filtered_posA, filtered_posB, targetA, targetB)

        if distance < 0.3:
            controller.send_speed_command(0)  # Dừng xe
            rospy.loginfo("Target reached. Stopping vehicle.")
            break  
        else:
            controller.send_speed_command(speed)
            
            angle_to_target = math.degrees(math.atan2(filtered_posB - targetB, targetA - filtered_posA))
            if((abs(angle_to_target) > yaw_value_new) and (yaw_value_new > 0)):
                right_or_left = True #Left
                rotate_angle_value = -(angle_to_target - yaw_value_new)
            elif((abs(angle_to_target) < yaw_value_new) and (yaw_value_new > 0)):
                right_or_left = False #Right
                rotate_angle_value = yaw_value_new - angle_to_target
            elif(abs(angle_to_target) < abs(yaw_value_new) and yaw_value_new < 0 ):
                right_or_left = False #Right
                rotate_angle_value = angle_to_target - (yaw_value_new)
            elif(abs(angle_to_target) > abs(yaw_value_new) and yaw_value_new < 0 ):
                right_or_left = True #Left
                rotate_angle_value = yaw_value_new - angle_to_target
            angle_diff = (yaw_value_new) # 90 - gia tri yaw dang xoay

            if(right_or_left == True):
                if(angle_diff > 90):
                    rotate_angle_value = (angle_diff) - angle_to_target
                    steer_value = convert_angle_to_steer_value(rotate_angle_value)
                else:
                    if(angle_diff > angle_to_target):
                        steer_value = convert_angle_to_steer_value(rotate_angle_value)
                    else:
                        steer_value = convert_angle_to_steer_value(rotate_angle_value)
            elif(right_or_left == False):
                if(angle_diff < 0):
                    rotate_angle_value = angle_to_target - (angle_diff)
                    steer_value = convert_angle_to_steer_value(-rotate_angle_value)
                elif(angle_diff > 0):
                    if(angle_diff < angle_to_target):
                        steer_value = convert_angle_to_steer_value(-rotate_angle_value)
                    else:
                        steer_value = convert_angle_to_steer_value(rotate_angle_value)
            controller.send_steer_command(steer_value)        
        rate.sleep()


target_positions = read_edge_points_from_file()
def listener():
    rospy.init_node('combined_node', anonymous=True)
    
    rospy.Subscriber('/automobile/localisation', localisation, localisation_callback)
    rospy.Subscriber('/automobile/IMU', IMU, imu_callback)

    kalman_thread = threading.Thread(target=kalman_thread_func)
    kalman_thread.daemon = True
    kalman_thread.start()

    for target_pos in target_positions:
        control_vehicle(target_pos)

    rospy.spin()

if __name__ == '__main__':
    listener()
