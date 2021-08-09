#!/usr/bin/env python

# Copyright 2021 Lum Kin Yun
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This is a sample ROS Webots controlled to control TurtleBot3 Burger for education purpose.
"""

import rospy
# from rospy.core import loginfo
# from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from controller import Robot
import os
import math

NAME = "Burger"
WHEEL_RADIUS = 0.033          # meter
WHEEL_SEPARATION = 0.160      # meter (BURGER : 0.160, WAFFLE : 0.287)
TURNING_RADIUS = 0.080        # meter (BURGER : 0.080, WAFFLE : 0.1435)
ROBOT_RADIUS = 0.105          # meter (BURGER : 0.105, WAFFLE : 0.220)
ENCODER_MIN = -2147483648     # raw
ENCODER_MAX = 2147483648      # raw
# m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
MAX_LINEAR_VELOCITY = (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60)
MAX_ANGULAR_VELOCITY = (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       # rad/s
MIN_LINEAR_VELOCITY = -MAX_LINEAR_VELOCITY
MIN_ANGULAR_VELOCITY = -MAX_ANGULAR_VELOCITY

# webots max motor speed is 2*PI one rotation
WEBOTS_MOTOR_MAX_VELOCITY = 2 * math.pi
INFINITY = float('+inf')

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

lidar = robot.getDevice("LDS-01")
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

lidar_main_motor = robot.getDevice("LDS-01_main_motor")
lidar_secondary_motor = robot.getDevice("LDS-01_secondary_motor")
lidar_main_motor.setPosition(INFINITY)
lidar_secondary_motor.setPosition(INFINITY)
lidar_main_motor.setVelocity(30.0)
lidar_secondary_motor.setVelocity(60.0)

right_motor = robot.getDevice("right wheel motor")
left_motor = robot.getDevice("left wheel motor")
right_motor.setPosition(INFINITY)
left_motor.setPosition(INFINITY)
right_motor.setVelocity(0.0)
left_motor.setVelocity(0.0)

# lidar_width = lidar.getHorizontalResolution()
# lidar_max_range = lidar.getMaxRange()

# velocity = 0
message = ''

rospy.loginfo('Initializing ROS: connecting to ' +
              os.environ['ROS_MASTER_URI'])
robot.step(TIME_STEP)

# Listener
def speedConstrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def cmdVelCallback(data):
    global message
    global left_motor
    global right_motor

    lin_vel = speedConstrain(
        data.linear.x,  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
    ang_vel = speedConstrain(
        data.angular.z, MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)

    left_wheel_lin_vel = lin_vel - (ang_vel * WHEEL_SEPARATION / 2)
    right_wheel_lin_vel = lin_vel + (ang_vel * WHEEL_SEPARATION / 2)
    
    left_wheel_ang_vel = speedConstrain(
        left_wheel_lin_vel / WHEEL_RADIUS, -WEBOTS_MOTOR_MAX_VELOCITY, WEBOTS_MOTOR_MAX_VELOCITY) 
    right_wheel_ang_vel = speedConstrain(
        right_wheel_lin_vel / WHEEL_RADIUS, -WEBOTS_MOTOR_MAX_VELOCITY, WEBOTS_MOTOR_MAX_VELOCITY)

    message = f'left_wheel_lin_vel: {str(left_wheel_lin_vel)} angular velocity: {str(left_wheel_ang_vel)}'

    left_motor.setVelocity(left_wheel_ang_vel)
    right_motor.setVelocity(right_wheel_ang_vel)

rospy.init_node('listener', anonymous=True)
rospy.loginfo('Subscribing to "/cmd_vel" topic')
rospy.Subscriber('/cmd_vel', Twist, cmdVelCallback)

# Publisher
def readLidar(lidar):
    laserData = LaserScan()
    laserData.header.stamp = rospy.Time.now()
    laserData.header.frame_id = 'base_scan'
    laserData.angle_increment = 2*math.pi / lidar.getHorizontalResolution()
    laserData.scan_time = 0
    laserData.time_increment = 0
    laserData.angle_min = 0
    laserData.angle_max = 2 * math.pi - laserData.angle_increment
    laserData.range_min = lidar.getMinRange()
    laserData.range_max = lidar.getMaxRange()
    laserData.ranges = lidar.getRangeImage()
    laserData.intensities = [0]*360
    return laserData

    # std_msgs/Header header
    #     uint32 seq
    #     time stamp = scan->header.stamp = ros::Time::now();
    #     string frame_id = 'base_scan'
    # float32 angle_min = 0
    # float32 angle_max = 2 * math.PI - angle_increment
    # float32 angle_increment = 2*math.PI / lidar.getHorizontalResolution()
    # float32 time_increment = 0
    # float32 scan_time = 0
    # float32 range_min = lidar.getMinRange()
    # float32 range_max = lidar.getMaxRange()
    # float32[] ranges = lidar.getRangeImage()
    # float32[] intensities = [360]


pub = rospy.Publisher('scan', LaserScan, queue_size=10)
rospy.loginfo('Publishing the topic "sensor"')
rospy.loginfo('Running the control loop')

while robot.step(TIME_STEP) != -1 and not rospy.is_shutdown():
    lidar_values = readLidar(lidar)
    pub.publish(lidar_values)
    # print('Published sensor value: ', sensor.getValue())

    if message:
        # print(message)
        # rospy.loginfo(message)
        message = ''
    # left.setVelocity(velocity)
    # right.setVelocity(velocity)
