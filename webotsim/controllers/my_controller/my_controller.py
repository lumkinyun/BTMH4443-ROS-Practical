from controller import Robot, Motor, Lidar
import math

TIME_STEP = 64
BASE_SPEED = 1.5
INFINITY = float('+inf')


def gaussian(x,  mu, sigma):
    return (1.0 / (sigma * math.sqrt(2.0 * math.pi))) * math.exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma))


robot = Robot()

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

lidar_width = lidar.getHorizontalResolution()
lidar_max_range = lidar.getMaxRange()

braitenberg_coefficients = [0] * lidar_width
for i in range(lidar_width):
    braitenberg_coefficients[i] = 6 * \
        gaussian(i, lidar_width / 4, lidar_width / 12)

while robot.step(TIME_STEP) != -1:
    left_speed = BASE_SPEED
    right_speed = BASE_SPEED

    lidar_values = lidar.getRangeImage()
    horizontal_res = lidar.getPointCloud( data_type='list')
    # print(f'lidar width {lidar_values}')
    maxVel = left_motor.getMaxVelocity()
    print(f'horizontal {maxVel}')

    for i in range(int(0.25 * lidar_width), int(0.5 * lidar_width), 1):
        j = int(lidar_width - i - 1)
        k = int(i - 0.25 * lidar_width)
        leftTemp = braitenberg_coefficients[k] * ((1.0 - lidar_values[i] / lidar_max_range) - (1.0 - lidar_values[j] / lidar_max_range))
        rightTemp =  braitenberg_coefficients[k] * ((1.0 - lidar_values[j] / lidar_max_range) - (1.0 - lidar_values[i] / lidar_max_range))
        if (not (leftTemp != leftTemp)):
            left_speed += leftTemp
        if (not (rightTemp != rightTemp)):
            right_speed += rightTemp


    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    # pass
    # # read sensors
    # left_dist = left_sensor.getValue()
    # right_dist = right_sensor.getValue()

    # # compute behavior (user functions)
    # left = compute_left_speed(left_dist, right_dist)
    # right = compute_right_speed(left_dist, right_dist)

    # # actuate wheel motors
    # left_motor.setVelocity(left)
    # right_motor.setVelocity(right)
