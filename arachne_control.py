import maestro
import time
import numpy as np

m_limits = (
    
    # Right Side
    
    (992, 1552, 1, 1122.25),  # min, max, direction, nominal
    (1136, 1744, -1, 1277.5),
    (1328, 1888, -1, 1815.25),
    
    (496, 928, -1, 848.25),
    (1232, 1872, -1, 1405.75),
    (1568, 1936, 1, 1901.25),

    (1056, 1856, 1, 1250.0),
    (1312, 1920, -1, 1441.75),
    (1488, 2048, -1, 1119.0),
    
    # Left Side
    
    (944, 1632, -1, 1468.25),  #  9
    (720, 1264, 1, 1195.25),   # 10
    (848, 1200, 1, 1871.75),   # 11
    
    (944, 1376, -1, 1405.25),
    (752, 1200, 1, 1214.25),
    (1008, 1488, 1, 1556.75),
    
    (880, 1600, -1, 1537.0),
    (704, 1104, 1, 1147.0),
    (688, 1248, 1, 780.75),
    )

m_limits_scale = 4   # multiply all of the above for what should actually be written to the Maestro (not sure why)
MLS = m_limits_scale

CURR_SERVO_POS = [0 for i in m_limits]
AVE_SERVO_POS = [servo_setting[3] for servo_setting in m_limits]  # Start at nominal for each

print(CURR_SERVO_POS)
print(AVE_SERVO_POS)


servo = maestro.Controller(device=12)

ACCEL = 5

# Set software limits to be the same as the hardware limits
for i in range(len(m_limits)):
    #servo.setRange(i, (m_limits[i][0]+1)*MLS, (m_limits[0][1]-1)*MLS)
    servo.setAccel(i, ACCEL)
    
def wait_while_moving(channel=0):
    #print("waiting")
    while servo.isMoving(channel):
        time.sleep(0.1)
        #print(servo.getPosition(channel))
    #print("done waiting")
    return
        
#print(servo.getMin(0), servo.getMax(0))
#servo.setAccel(0,5)      #set servo 0 acceleration to 4


#servo.setTarget(1,m_limits[1][0]*MLS)  #set servo to move to center position
#wait_while_moving(1)

#servo.setTarget(1,m_limits[1][1]*MLS)  #set servo to move to center position
#wait_while_moving(1)

def reset_all_joints():
    for servo_num in range(len(m_limits)):
        joint_nom = int(m_limits[servo_num][3]*MLS)
        servo.setTarget(servo_num, joint_nom)

def set_all_joints_temp():
    for servo_num in range(len(m_limits)):
        joint_nom = int(m_limits[servo_num][3]*MLS-250)
        servo.setTarget(servo_num, joint_nom)


def get_all_joints_pos():
    for servo_num in range(len(m_limits)):
        CURR_SERVO_POS[servo_num] = servo.getPosition()

def is_moving():
    squared_sum = 0
    for servo_num in range(len(m_limits)):
        squared_sum = (CURR_SERVO_POS - AVE)

import sys

reset_all_joints()
time.sleep(3)
#set_all_joints_temp()
#time.sleep(3)
#reset_all_joints()
sys.exit()

def move_joint(leg=0, joint=0, amount=50, wait=True):  # Amount goes 0 to 100
    servo_num = leg*3+joint
        
    joint_min = m_limits[servo_num][0]*MLS
    joint_max = m_limits[servo_num][1]*MLS
    joint_range = joint_max - joint_min
    joint_dir = m_limits[servo_num][2]
    
    if joint_dir > 0:
        target = int(joint_min + (amount/100)*joint_range)
    else:
        target = int(joint_max - (amount/100)*joint_range)
    print(joint_min, joint_max, joint_dir, target)
    
    servo.setTarget(servo_num, target)
    if wait: wait_while_moving(servo_num)

def move_ankle_up(leg=0, amount=50, wait=False):
    move_joint(leg, 0, amount, wait)

def move_leg_up(leg=0, amount=50, wait=False):
    move_joint(leg, 1, amount, wait)  
    
def move_leg_fwd(leg=0, amount=50, wait=False):
    move_joint(leg, 2, amount, wait)
    




def legs_set_position(legs, forward=50, height=50, ankle=50, delay=0.0):
    for leg in legs:
        move_ankle_up(leg, ankle)
        move_leg_up(leg, height)
        move_leg_fwd(leg, forward)
        time.sleep(delay)   
      
def legs_step_fwd(legs, amount=0, height=50, delay=0.5): # amount is 0 to 100
    for leg in legs:
        move_ankle_up(leg,100)
        move_leg_up(leg,100)
        move_leg_fwd(leg, amount)
        time.sleep(delay)
        move_ankle_up(leg, 70)
        move_leg_up(leg, height)
        time.sleep(delay)

def legs_step_bck(legs, delay=0.5):
    for leg in legs:
        move_ankle_up(leg,100)
        move_leg_up(leg,100)
        move_leg_fwd(leg, 0)
        time.sleep(delay)
        move_ankle_up(leg, 50)
        move_leg_up(leg, 50)
        time.sleep(delay)


def legs_up(legs, amount = 100, delay=0.5):
    for leg in legs:
        move_ankle_up(leg,amount)
        move_leg_up(leg,amount)
        time.sleep(delay)
        
def legs_down(legs, amount = 10, delay=0.5):
    for leg in legs:
        move_ankle_up(leg)
        move_leg_up(leg,amount)
        time.sleep(delay)

def legs_move_back(legs, amount=0):
    for leg in legs:
            move_leg_fwd(leg, amount)        

def legs_move_fwd(legs, amount=0):
    for leg in legs:
            move_leg_fwd(leg, amount)        

    
 
def walking_5(count=1):
    for i in range(count):
        legs1 = (1,4,2,5)
        legs2 = (2,3)   # 75, (2,3) almost falls over; 50, (2,3) is ok
        legs_step_fwd((0,3,1,4,2,5), 40)  # 25 falls over; 50 is ok. 75 is ok; 100 is ok
        time.sleep(0.5)
        #legs_up(legs2) #, 50)
        # time.sleep(1)
        legs_set_position(legs2, 50, 100, 0)
        time.sleep(0.5)
        legs_move_fwd(legs1,75)   # 85, falls over
        time.sleep(0.5)
        #legs_step_fwd((0,3,1,4,2,5), 50)  # 25 falls over; 50 is ok. 75 is ok; 100 is ok
    legs_step_fwd((0,3,1,4,2,5), 40)  # 25 falls over; 50 is ok. 75 is ok; 100 is ok
        
        
    
    
#walking_5(4)
#legs_reset((0,5,1,4,2,3))
#time.sleep(1)
# for count in range(3):
#     legs_step_fwd((0, 5, 1, 4, 2, 3), 50, delay=1)         
#     time.sleep(2)
#     #legs_step_fwd((1, 4, ), 0)
#     legs_move_fwd((0,5,1,4,2,3), 0)
#     legs_step_fwd((0, 5, 1, 4, 2, 3), 50, delay=1)         
#     #legs_step_bck((0,))

#legs = (0,2,4)
#legs_up(legs)
#time.sleep(1)
#legs_down(legs)






import numpy as np

def inverse_kinematics(x, y, L1, L2):
    """
    Computes theta1 and theta2 given foot position (x, y) and link lengths (L1, L2).
    Angles are returned in degrees.
    """
    r2 = x**2 + y**2  # Squared distance to foot position
    
    # Compute theta2 using the law of cosines
    cos_theta2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(cos_theta2) > 1:
        raise ValueError("Position out of reach")
    theta2 = np.arccos(cos_theta2)
    
    # Compute theta1
    alpha = np.arctan2(y, x)
    beta = np.arccos((r2 + L1**2 - L2**2) / (2 * L1 * np.sqrt(r2)))
    theta1 = alpha - beta
    
    return np.degrees(theta1), np.degrees(theta2)

# Example usage
L1, L2 = 89, 122  # mm
x, y = 127, 140 # mm
theta1p, theta2p = inverse_kinematics(x, y, L1, L2)
print(f"Theta1p: {theta1p:.2f} degrees, Theta2p: {theta2p:.2f} degrees")

step_size = -20
theta1, theta2 = inverse_kinematics(x+step_size, y, L1, L2)
print(f"Theta1: {theta1:.2f} degrees, Theta2: {theta2:.2f} degrees")

delta_theta1 = theta1 - theta1p
delta_theta2 = theta2 - theta2p
print("delta_theta:", delta_theta1, delta_theta2)

scale_theta1 = 100 / 90 # steps / degrees
scale_theta2 = 100 / 50 # steps / degrees

delta_theta1_steps = int(round(scale_theta1 * delta_theta1,0))
delta_theta2_steps = int(round(scale_theta2 * delta_theta2,0))

print("delta_theta_steps", delta_theta1_steps, delta_theta2_steps)



# Walk


# Test angles
legs0 = (2,3)
legs1 = (1,4)
legs2 = (0,5)
#legs2 = (2,3)   # 75, (2,3) almost falls over; 50, (2,3) is ok
#legs_step_fwd((0,3,1,4,2,5), 50)  # 25 falls over; 50 is ok. 75 is ok; 100 is ok

#legs_step_fwd(legs1+legs2, 50)
#time.sleep(1.0)
for count in range(5):

    legs_step_fwd(legs1, 100)
    time.sleep(1.0)
    legs_step_fwd(legs0, 25)
    time.sleep(1.0)
    legs_step_fwd(legs2, 10)
    time.sleep(1.0)

    legs_up(legs1)
    time.sleep(1.0)

    #legs_set_position(legs0, 0, 50, 50)
    legs_set_position(legs0, 25, 50-delta_theta1_steps, 50-delta_theta2_steps)
    legs_set_position(legs2, 25)
    time.sleep(1.0)

    legs_down(legs1)

    time.sleep(2.0)


#servo.setSpeed(1,10)     #set speed of servo 1
x = servo.getPosition(0) #get the current position of servo 1
print(x)
print(servo.isMoving(0))
servo.close()