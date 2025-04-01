# Arachne Control and Walking

# ToDo:
# * Have all the relative_xyz movements wait till one be execute.  Right now, each leg is separate.
# * Use a function like Maestro's IsMoving to compare desired setpoint to what was actually achieved.
#   In this way, you can check for objects in the way/ changes in floor, and maybe self-calibration
#   Check range of x,y,z that each leg can achieve, store in a table?
# * Store each joint angle as it is written so that the inverse-functions for relative movement are more exact
# * Add a calibration point (or maybe 2) to make moving more accurate
# * #FIXME: There is some error in relative motion on legs 2 and 3 such that
#   they go up as you do more relative moves.  Or was it just a delay problem?
# * #FIXME: wait function is really slow, but the sleeps are not scalable



import maestro
import time
import numpy as np
import sys

DEBUG = True


# Define Motor limits and characteristics

m_limits = [
    
    # Right Side

    # FIXME: Not using min/max right now.  Do I need to?
    
    [992, 1552, 1, 1122.25, 90, 1773.5, 999],  # min, max, direction, nominal [0 degrees], setting [calibrated], calibrated angle. slope [calc'd later]
    [1136, 1744, -1, 1277.5, -45, 1512.50, 999],
    [1328, 1888, -1, 1815.25, 45, 1458.25, 999],
    
    [496, 928, -1, 848.25, 38, 646.25, 999],
    [1232, 1872, -1, 1405.75, -45, 1643.0, 999],
    [1568, 1936, 1, 1901.25, 45, 2189.5, 999 ],

    [1056, 1856, 1, 1250.0, 90, 1976.25, 999],
    [1312, 1920, -1, 1441.75, -45, 1668.0, 999],
    [1488, 2048, -1, 1119.0, -45, 1618.75, 999],
    
    # Left Side
    
    [944, 1632, -1, 1468.25, 90, 832.0, 999],  #  9
    [720, 1264, 1, 1195.25, -45, 929.25, 999],   # 10
    [848, 1200, 1, 2488.25,-45, 1993.0, 999 ],   # 11
    
    [944, 1376, -1, 1435.25, 90, 784.0, 999],
    [752, 1200, 1, 1214.25, -45, 891.75, 999],
    [1008, 1488, 1, 1556.75, 45, 1937.25, 999],
    
    [880, 1600, -1, 1500.0, 90, 803.0, 999],
    [704, 1104, 1, 1147.0, -45, 799.25, 999],
    [688, 1248, 1, 780.75, 45, 1139.5, 999],
    ]


L1, L2 = 85, 123  # mm  #FIXME: Measure better

m_limits_old = [
    
    # Right Side

    # FIXME: Not using min/max right now.  Do I need to?
    
    [992, 1552, 1, 1122.25, 45, 1391.25, 999],  # min, max, direction, nominal [0 degrees], setting [calibrated], calibrated angle. slope [calc'd later]
    [1136, 1744, -1, 1277.5, -45, 1512.50, 999],
    [1328, 1888, -1, 1815.25, 45, 1458.25, 999],
    
    [496, 928, -1, 848.25, 45, 646.25, 999],
    [1232, 1872, -1, 1405.75, -45, 1643.0, 999],
    [1568, 1936, 1, 1901.25, 45, 2189.5, 999 ],

    [1056, 1856, 1, 1250.0, 45, 1529.75, 999],
    [1312, 1920, -1, 1441.75, -45, 1668.0, 999],
    [1488, 2048, -1, 1119.0, -45, 1618.75, 999],
    
    # Left Side
    
    [944, 1632, -1, 1468.25, 45, 1286.50, 999],  #  9
    [720, 1264, 1, 1195.25, -45, 769.25, 999],   # 10
    [848, 1200, 1, 2488.25,-45, 1993.0, 999 ],   # 11
    
    [944, 1376, -1, 1405.25, 45, 1193.0, 999],
    [752, 1200, 1, 1214.25, -45, 891.75, 999],
    [1008, 1488, 1, 1556.75, 45, 1937.25, 999],
    
    [880, 1600, -1, 1537.0, 45, 1211.75, 999],
    [704, 1104, 1, 1147.0, -45, 799.25, 999],
    [688, 1248, 1, 780.75, 45, 1139.5, 999],
    ]



############# Other Definitions ################

ACCEL = 45   # Global acceleration setting for all servos
SERVO_SPEED = 100
POS_AVERAGE_COUNT = 3   # Number of position readings to average together
m_limits_scale = 4   # multiply all of the above for what should actually be written to the Maestro (not sure why)
MLS = m_limits_scale
MOVING_THRESH = 50  # How far away is "close-enough" to be still-moving

CURR_SERVO_POS = [0 for i in m_limits]
AVE_SERVO_POS = [servo_setting[3]*MLS for servo_setting in m_limits]  # Start at nominal for each
NEXT_SERVO_POS = [0 for i in m_limits]

#SERVO_SCALE = 476.25*MLS/90.0   # steps / degrees


# Global servo control
servo = maestro.Controller(device=12)

# Set software limits to be the same as the hardware limits, calculate slopes
for i in range(len(m_limits)):
    #servo.setRange(i, (m_limits[i][0]+1)*MLS, (m_limits[0][1]-1)*MLS)
    servo.setAccel(i, ACCEL)
    servo.setSpeed(i, SERVO_SPEED)
    cal1_angle = 0  # angle of first cal point
    cal1_setting = m_limits[i][3]
    cal2_angle = m_limits[i][4]
    cal2_setting = m_limits[i][5]
    slope = (cal2_setting - cal1_setting)/(cal2_angle - cal1_angle)

    m_limits[i][6] = slope*MLS


    print(m_limits[i])

#sys.exit()



##### Functions ##########
    
def reset_ave_servo_pos():
    global AVE_SERVO_POS
    AVE_SERVO_POS = [0 for i in m_limits]   #FIXME: SHould this just update to current position???

def get_all_joints_pos(ave_count = POS_AVERAGE_COUNT):
    global CURR_SERVO_POS, AVE_SERVO_POS
    for servo_num in range(len(m_limits)):
        curr_pos = servo.getPosition(servo_num)
        ave_pos = (AVE_SERVO_POS[servo_num]*ave_count + curr_pos)/(ave_count+1)
        #print("curr_pos", curr_pos)
        CURR_SERVO_POS[servo_num] = curr_pos
        AVE_SERVO_POS[servo_num] = ave_pos

def update_ave_joints_pos(ave_count = POS_AVERAGE_COUNT):
    global CURR_SERVO_POS, AVE_SERVO_POS
    for servo_num in range(len(m_limits)):
        curr_pos = servo.getPosition(servo_num)
        ave_pos = (AVE_SERVO_POS[servo_num]*ave_count + curr_pos)/(ave_count+1)
        #print("curr_pos", curr_pos)
        CURR_SERVO_POS[servo_num] = curr_pos
        AVE_SERVO_POS[servo_num] = ave_pos



#######  Direct Joint control functions ##############

def reset_all_joints():
    reset_ave_servo_pos()
    for servo_num in range(len(m_limits)):
        joint_nom = int(m_limits[servo_num][3]*MLS)
        servo.setTarget(servo_num, joint_nom)


def set_all_joints(pos_joints):
    reset_ave_servo_pos()
    for servo_num in range(len(m_limits)):
        #joint_nom = int(m_limits[servo_num][3]*MLS)
        joint_nom = pos_joints[servo_num]
        servo.setTarget(servo_num, joint_nom)


def is_moving(thresh = MOVING_THRESH):
    global CURR_SERVO_POS, AVE_SERVO_POS
    #update_ave_joints_pos()
    get_all_joints_pos()
    squared_sum = 0
    for servo_num in range(len(m_limits)):
        squared_sum += (CURR_SERVO_POS[servo_num] - AVE_SERVO_POS[servo_num])**2
    moving_amount = np.sqrt(squared_sum)
    if moving_amount >= thresh:
        return True
    else:
        return False 


def wait_while_moving(thresh = MOVING_THRESH, delay = 0.05):
    moving = is_moving(thresh)
    while moving:
        time.sleep(delay)
        moving = is_moving(thresh)
        

def move_joint_rel(leg=0, joint=0, degrees=10, wait=True):
    reset_ave_servo_pos()
    servo_num = leg*3+joint
    curr_pos = CURR_SERVO_POS[servo_num]
    angle_scale = m_limits[servo_num][6]  #476.25*4/90  # steps / degrees
    joint_dir = m_limits[servo_num][2]
    new_pos = int(curr_pos + degrees*angle_scale) #joint_dir*degrees*angle_scale)
    servo.setTarget(servo_num, new_pos)
    if wait: wait_while_moving()


def move_joint(leg=0, joint=0, amount=50, wait=True):  # Amount goes 0 to 100
    reset_ave_servo_pos()
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
    if wait: wait_while_moving()


def move_joint_angle(leg=0, joint=0, angle=0, wait=False, wait_till_execute = False): 
    reset_ave_servo_pos()
    servo_num = leg*3+joint
    
    joint_dir = m_limits[servo_num][2]
    joint_nom = int(m_limits[servo_num][3]*MLS)   # this corresponds to zero degrees

    angle_scale = m_limits[servo_num][6]  #476.25*4/90  # steps / degrees
    
    target = int(joint_nom + angle*angle_scale) #joint_dir*angle*SERVO_SCALE)

    print("leg, joint, angle, joint_dir, joint_nom, target:", leg, joint, angle, joint_dir, joint_nom, target)
    if not wait_till_execute:
        servo.setTarget(servo_num, target)
    else:
        NEXT_SERVO_POS[servo_num] = target
    if not wait_till_execute and wait: wait_while_moving()


def execute_move_joint_angle(wait = False):
    print("Executing")
    reset_ave_servo_pos()
    for servo_num in range(len(m_limits)):
        #joint_nom = int(m_limits[servo_num][3]*MLS)
        joint_nom = NEXT_SERVO_POS[servo_num]
        if joint_nom > 0:
            servo.setTarget(servo_num, joint_nom)
            NEXT_SERVO_POS[servo_num] = 0
    if wait: wait_while_moving()
        

########### Calculate Angles / Positions of legs #################

def inverse_kinematics(x, y, z, L1, L2):
    """
    Computes theta0, theta1, and theta2 given foot position (x, y, z) and link lengths (L1, L2).
    Angles are returned in degrees.
    """
    # Compute theta0 (rotation in the y-x plane)
    theta0 = np.degrees(np.arctan2(y, x))
    
    z = -z  # RKD
    
    # Compute the projected distance in the y-z plane
    r = np.sqrt(y**2 + x**2)
    s = np.sqrt(r**2 + z**2)  # Total length from base to foot
    
    # Compute theta2 using the law of cosines
    cos_theta2 = (s**2 - L1**2 - L2**2) / (2 * L1 * L2)
    #FIXME: need to tweak these out of reach things
    if abs(cos_theta2) > 1:
        raise ValueError("Position out of reach")
    theta2 = np.degrees(-np.arccos(cos_theta2)) + 90 # RKD
    #theta2 = -theta2  # Adjust to match forward kinematics convention
    # Compute theta1 using the law o#f cosines
    cos_beta = (s**2 + L1**2 - L2**2) / (2 * L1 * s)
    if abs(cos_beta) > 1:
        raise ValueError("Position out of reach")
    beta = np.arccos(cos_beta) #+ np.pi/2  # Axis incorrectly specified???
    alpha = np.arctan2(z, r)
    theta1 = -np.degrees(alpha - beta)
    
    return theta0, theta1, theta2


def forward_kinematics(theta0, theta1, theta2, L1, L2):
    """
    Computes foot position (y, x, z) given joint angles (theta0, theta1, theta2) and link lengths (L1, L2).
    Angles are assumed to be in degrees.
    """
    theta2 = theta2 - 90  # RKD
    theta0, theta1, theta2 = np.radians([theta0, theta1, theta2])
    
    # Compute position in the sagittal plane (y'-z plane)
    r = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    z = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    
    # Rotate into 3D space
    x = r * np.cos(theta0)
    y = r * np.sin(theta0)
    
    return x, y, z


def angle_from_steps(servo_num, target):
    joint_dir = m_limits[servo_num][2]
    joint_nom = int(m_limits[servo_num][3]*MLS)   # this corresponds to zero degrees

    #target = int(joint_nom + joint_dir*angle*SERVO_SCALE)
    angle_scale = m_limits[servo_num][6]  #476.25*4/90  # steps / degrees
    
    angle = (target - joint_nom)/angle_scale #(joint_dir*SERVO_SCALE)
    return angle


def steps_from_angle(servo_num, angle):
    joint_dir = m_limits[servo_num][2]
    joint_nom = int(m_limits[servo_num][3]*MLS)   # this corresponds to zero degrees
    angle_scale = m_limits[servo_num][6]  #476.25*4/90  # steps / degrees
    
    target = int(joint_nom + angle*angle_scale) #joint_dir*angle*SERVO_SCALE)
    #angle = (target - joint_nom)/(joint_dir*SERVO_SCALE)
    return target





################ Multiple-Leg movement functions #################


def legs_move_xyz(legs, x, y, z, wait_end=False):
    reset_ave_servo_pos()
    theta0, theta1, theta2 = inverse_kinematics(x, y, z, L1, L2)
    legs_move_angles(legs, theta0, theta1, theta2, wait_end)


def legs_move_angles(legs, theta0, theta1, theta2, wait_end=False):
    reset_ave_servo_pos()
    for leg in legs:
        move_joint_angle(leg, 0, theta2, wait_till_execute=True)  # note thetaX and joint are opposite.  Just because.
        move_joint_angle(leg, 1, theta1, wait_till_execute=True)
        move_joint_angle(leg, 2, theta0, wait_till_execute=True)
    execute_move_joint_angle()    
    if wait_end: wait_while_moving()


def legs_step_angles(legs, theta0, theta1, theta2, step_angle=30, wait_end=False, wait_each=False):
    reset_ave_servo_pos()
    for leg in legs:
        #theta0, theta1, theta2 = inverse_kinematics(x, y, z, L1, L2)
        #servo_num = leg*3+joint
        move_joint_angle(leg, 1, theta1+step_angle)
        time.sleep(0.25)
        move_joint_angle(leg, 2, theta0, False)
        move_joint_angle(leg, 0, theta2, False)  # note thetaX and joint are opposite.  Just because.
        time.sleep(0.25)
        move_joint_angle(leg, 1, theta1, False)
        time.sleep(0.25)
    if wait_end: wait_while_moving()


def legs_move_relative(legs, delta_x, delta_y, delta_z, wait_end=False):
    reset_ave_servo_pos()
    get_all_joints_pos()
    new_position = CURR_SERVO_POS.copy()  #FIXME: Do all movements at once
    for leg in legs:
        theta2 = angle_from_steps(3*leg+0,CURR_SERVO_POS[3*leg+0])
        theta1 = angle_from_steps(3*leg+1,CURR_SERVO_POS[3*leg+1])
        theta0 = angle_from_steps(3*leg+2,CURR_SERVO_POS[3*leg+2])
        x0, y0, z0 = forward_kinematics(theta0, theta1, theta2, L1, L2)
        x, y, z = x0 + delta_x, y0 + delta_y, z0 + delta_z
        legs_move_xyz((leg,), x, y, z, False)
    if wait_end: wait_while_moving()


def legs_step_relative(legs, delta_x, delta_y, delta_z, step_z=50, wait_end=False):
    reset_ave_servo_pos()
    get_all_joints_pos()
    new_position = CURR_SERVO_POS.copy()  #FIXME: Do all movements at once
    for leg in legs:
        theta2 = angle_from_steps(3*leg+0,CURR_SERVO_POS[3*leg+0])
        theta1 = angle_from_steps(3*leg+1,CURR_SERVO_POS[3*leg+1])
        theta0 = angle_from_steps(3*leg+2,CURR_SERVO_POS[3*leg+2])
        x0, y0, z0 = forward_kinematics(theta0, theta1, theta2, L1, L2)
        x, y, z = x0 + delta_x, y0 + delta_y, z0 + delta_z
        legs_move_xyz((leg,), x0, y0, z0+step_z, False)
        time.sleep(0.1)
        legs_move_xyz((leg,), x, y, z0+step_z, False)
        time.sleep(0.1)
        legs_move_xyz((leg,), x, y, z, False)
        time.sleep(0.1)
        

    
############################################################################




print("Starting")

# Get to default walking position
# reset_all_joints()
# wait_while_moving()
# time.sleep(1)
# legs_move_angles((0,5,), 60, 0, -20, False)
# legs_move_angles((2,3,), -60, 0, -20, False)
# legs_move_angles((1,4,), 0, 0, -20, False)
# time.sleep(1)
# legs_move_relative((0,1,2,3,4,5), 0,0,60)
# time.sleep(1)
# update_ave_joints_pos()
# print(CURR_SERVO_POS)

# pos_walking_default = [3343, 4079, 5991, 4537, 4593, 7605, 3854, 4736, 5746, 7017, 5810, 8683, 6765, 5886, 6227, 7292, 5617, 4393]

#sys.exit()

#legs_move_angles((0,5,1,4,2,3), 0, 0, 0)

#sys.exit()


for count in range(3):

    legs_step_angles((5,0,), 55, 10, -5)
    legs_step_angles((4,1), 25, 10, -25)
    legs_step_angles((3,2,), -15, 10, -25)
    wait_while_moving()
    #time.sleep(1)

    legs_move_relative((0,1,2,3,4,5), 0, -60, 0, False)  # was 0, -70, 0
    wait_while_moving()
    



# Cleanup

servo.close()