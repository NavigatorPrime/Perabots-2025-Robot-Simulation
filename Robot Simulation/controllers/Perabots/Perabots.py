from controller import Robot
import math

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice('rotational_motor_1')
right_motor = robot.getDevice('rotational_motor_2')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Lidar
lidar = robot.getDevice("lidar")
lidar.enable(TIME_STEP)

# GPS
gps = robot.getDevice("global")
gps.enable(TIME_STEP)

# Movement constants
FORWARD_SPEED = 5.7
TURN_SPEED = 6
Turn_threshold=0.21
Front_threshold=0.23

Learned_values=[]


# Movement functions
def move_forward():
    left_motor.setVelocity(FORWARD_SPEED)
    right_motor.setVelocity(FORWARD_SPEED)

def stop():
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

def turn_left():
    left_motor.setVelocity(TURN_SPEED)
    right_motor.setVelocity(-TURN_SPEED)

def turn_right():
    left_motor.setVelocity(-TURN_SPEED)
    right_motor.setVelocity(TURN_SPEED)


Total=0

# Turns the robot by a specific angle in degrees
def turn_robot(angle_deg):
       
    angular_velocity=6
    wheel_radius=0.04
    axle_length=0.12

    # Convert angle to radians
    angle_rad = math.radians(abs(angle_deg))

    # Time needed for the turn
    duration = angle_rad / angular_velocity

    # Calculate wheel speeds (opposite directions for in-place turn)
    wheel_speed = (angular_velocity * axle_length) / 2
    left_speed = -wheel_speed / wheel_radius
    right_speed = wheel_speed / wheel_radius

    # Reverse direction if angle is negative
    if angle_deg < 0:
        left_speed, right_speed = -left_speed, -right_speed

    # Set wheel speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # Turn for calculated time
    start_time = robot.getTime()
    while robot.step(TIME_STEP) != -1:
        if robot.getTime() - start_time >= duration:
            break

    # Stop motors
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)



runs=1
Stop_count=0
lap=1


def wait(duration_seconds):
    
    elapsed = 0
    while elapsed < duration_seconds:
        if robot.step(TIME_STEP) == -1:
            break
        elapsed += TIME_STEP / 1000.0


# Main loop
while robot.step(TIME_STEP) != -1:
    lidar_values = lidar.getRangeImage()
    Dis_list=[]
    gps_values = gps.getValues()
    current_pos = [gps_values[0], gps_values[1]]  # x, z
    distance=(gps_values[0]**2+gps_values[1]**2)**(1/2)
    
    # Divide lidar into 6 segments
    for i in range(0,541):
        Total=Total+lidar_values[i]
        if (i%90)==0 and i!=0:
            Mid_dis=Total/90
            Total=0
            Dis_list.append(Mid_dis)

    # Movement decisions
    if Dis_list[2]>=Front_threshold and Dis_list[3]>=Front_threshold:
        move_forward()
        pos=2
    elif (Dis_list[1]>=Turn_threshold) or (0<=Dis_list[1]<=0.01) or (lap==57): 
        current_angle=(1-pos)*30     
        turn_robot(current_angle)
        pos=1
    elif (Dis_list[0]>=Turn_threshold) or (lap==57):
        current_angle=(0-pos)*30
        turn_robot(current_angle)
        pos=0
    elif Dis_list[4]>=Turn_threshold:
        current_angle=(4-pos)*30
        turn_robot(current_angle)
        pos=4
        move_forward()
    elif Dis_list[5]>=Turn_threshold:
        current_angle=(5-pos)*30
        turn_robot(current_angle)
        pos=5
        move_forward()
    if distance>=0.699 and runs<3:
        current_angle=(1-pos)*30     
        turn_robot(current_angle)
        pos=1
        runs+=1

    # Adjust thresholds in specific zones
    if ((-0.77<current_pos[0]<-0.5)):
        Front_threshold=0.13
        Turn_threshold=0.15
    if (-0.371<current_pos[0]<-0.31) and (-0.73<current_pos[1]<-0.58):
        stop()
        Turn_threshold=0.22
        Front_threshold=0.3
        wait(0.01)
        move_forward()
        lap+=1
    if (-0.79<current_pos[0]<-0.6) and (-0.16<current_pos[1]<-0.13):
        stop()
        wait(0.1)
        Turn_threshold=0
        Front_threshold=2.8
        move_forward()
    if lap==43 or 56<=lap<=59:
        current_angle=(1-pos)*30     
        turn_robot(current_angle)
        pos=0
        lap+=1
    # End condition
    if lap==69:
        stop()
        break   


    

    

    

  
