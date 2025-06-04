# Arachne Control and Walking

# ToDo:
# * Relative and RelZero movements should be wrt an absolute x direction for all legs
# * Make leg x consistent for both left side and right side
#   <-- FIXME: for some directions, the leg x orientation needs to change (+x out is not always good).  Maybe x direction should always be consistent?
# * if leg is already in the right spot, don't step
# * Change walking to be multi-step: move fwd leg/legs; move body fwd; move other legs fwd; more body the rest of the way fwd
# X Use a function like Maestro's IsMoving to compare desired setpoint to what was actually achieved.
#   In this way, you can check for objects in the way/ changes in floor, and maybe self-calibration
#   Check range of x,y,z (self-calibrate) so that each leg can achieve, store in a table?
# X Have all the relative_xyz movements wait till one big execute.  Right now, each leg is separate.
# * Store each joint angle as it is written so that the inverse-functions for relative movement are more exact
# X Add a calibration point (or maybe 2) to make moving more accurate
# X There is some error in relative motion on legs 2 and 3 such that
#   they go up as you do more relative moves.  Or was it just a delay problem?
# X wait function is really slow, but the sleeps are not scalable
# X wait while moving is ot working.  separate thread?

#FIXME: the new motor is 995 instead of 996.  Doesn't move smoothly.  But only in Joystick mode???



#import maestro
import pygame
import maestro_extended
import time
import numpy as np
import sys

debug = False





# Define Motor limits and characteristics

m_limits = [
    
    # Right Side

    # FIXME: Not using min/max right now.  Do I need to?
    
    [608, 1920, 1, 831.25, 90, 1773.5, 999],  # min, max, direction, nominal [0 degrees], setting [calibrated], calibrated angle. slope [calc'd later]
    [544, 1152, -1, 863, 45, 54, 999],  # 1
    [1248, 1840, -1, 1492, -45, 1840, 999],  # 2
    
    [64, 1136, -1, 1040, 38, 646.25, 999],  # 3
    [608, 1328, -1, 1020, 45, 608.0, 999],  # 4
    [1568, 2128, 1, 1901.25, 45, 2073, 999 ],  # 5

    [800, 2096, 1, 1250.0, 90, 1976.25, 999],  # 6
    [608, 1344, -1, 1019, 45, 608.0, 999],  # 7
    [736, 1632, -1, 1270, 45, 736, 999], # 8
    
    # Left Side
    
    [704, 1904, -1, 1468.25, 90, 832.0, 999],  #  9
    [1248, 1968, 1, 1580, 45, 1968, 999],   # 10
    [1248, 2080, 1, 1550.00, 45, 2031, 999 ],   # 11
    
    [608, 1760, -1, 1615, 90, 784.0, 999], #12
    [1328, 2000, 1, 1556, 45, 2000, 999],  #13
    [1008, 1824, 1, 1556.75, 45, 1790, 999], #14
    
    [608, 1904, -1, 1734.0, 90, 803.0, 999], #15
    [1232, 1952, 1, 1504.0, 45, 1952, 999], #16
    [848, 1312, 1, 1156, -45, 848, 999], #17
    ]




m_limits_2 = [
    
    # Right Side

    # FIXME: Not using min/max right now.  Do I need to?
    
    [608, 1920, 1, 831, 90, 1773.5, 999],  # min, max, direction, nominal [0 degrees], setting [calibrated], calibrated angle. slope [calc'd later]
    [704, 1392, -1, 964, -45, 1392, 999],  # 1
    [1248, 1840, 1, 1380, -45, 1840, 999],  # 2
    
    [64, 1136, -1, 1040, 38, 646.25, 999],  # 3
    [800, 1840, -1, 1160, -45, 1643.0, 999],  # 4
    [1568, 2496, 1, 1901.25, 45, 2189.5, 999 ],  # 5

    [800, 2096, 1, 1250.0, 90, 1976.25, 999],  # 6
    [800, 1504, -1, 1022, -45, 1668.0, 999],  # 7
    [736, 1632, -1, 1320.0, -90, 1931.75, 999], # 8
    
    # Left Side
    
    [704, 1904, -1, 1468.25, 90, 832.0, 999],  #  9
    [1088, 1840, 1, 1600, -45, 929.25, 999],   # 10
    [1248, 2080, 1, 1470.00, -90, 974.75, 999 ],   # 11
    
    [608, 1760, -1, 1615, 90, 784.0, 999], #12
    [752, 1600, 1, 1414.25, -45, 891.75, 999], #13
    [1008, 2000, 1, 1556.75, 45, 1937.25, 999], #14
    
    [608, 1904, -1, 1734.0, 90, 803.0, 999], #15
    [1040, 1808, 1, 1427.0, -45, 1044, 999], #16
    [848, 1504, 1, 1296, -45, 848, 999], #17
    ]



L1, L2 = 85, 123  # mm  #FIXME: Measure better



############# Other Definitions ################

ACCEL = 20   # Global acceleration setting for all servos
SERVO_SPEED = 200
POS_AVERAGE_COUNT = 3   # Number of position readings to average together
m_limits_scale = 4   # multiply all of the above for what should actually be written to the Maestro (not sure why)
MLS = m_limits_scale
MOVING_THRESH = 10  # How far away is "close-enough" to be still-moving

CURR_SERVO_POS = [0 for i in m_limits]
AVE_SERVO_POS = [servo_setting[3]*MLS for servo_setting in m_limits]  # Start at nominal for each
NEXT_SERVO_POS = [0 for i in m_limits]

#SERVO_SCALE = 476.25*MLS/90.0   # steps / degrees



# Define legs for different directions
legs_directions = {
      0: [(5,0), (4,1), (3,2), 0, 1],  # (front legs), (mid legs), (back legs), step x_scale, step y_scale]
     45: [(0,1), (5,2), (4,3), 0.7, 0.7],
     90: [(1,), (0,2), (3,4,5), 1, 0],
    135: [(1,2), (3,0), (4,5), 0.7, -0.7],
    180: [(2,3), (4,1), (5,0), 0, -1],
     -45: [(4,5), (3,0), (2,1), -0.7, -0.7],
     -90: [(4,), (3,5), (2,1,0), -1, 0],
    -135: [(3,4), (2,5), (1,0), -0.7, +0.7],
}


# PS4 Buttons
ps4_button = {
    0: 'Cross',
    1: 'Circle',
    2: 'Square',
    3: 'Triangle',
    4: 'Share',
    5: 'PS Button',
    6: 'Options',
    7: 'L. Stick In',
    8: 'R. Stick In',
    9: 'Left Bumper', 
    10: 'Right Bumper',
    11: 'D-pad Up',
    12: 'D-pad Down',
    13: 'D-pad Left',
    14: 'D-pad Right',
    15: 'Touch Pad Click',
}




# Global servo control
servo = maestro_extended.ExtendedController(device=12, stall_timeout=0.5, check_interval=0.05)

# Set software limits to be the same as the hardware limits, calculate slopes
for i in range(len(m_limits)):
    servo.setRange(i, (m_limits[i][0])*MLS, (m_limits[i][1])*MLS)
    
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
    global AVE_SERVO_POS, CURR_SERVO_POS
    AVE_SERVO_POS = [0 for i in m_limits]   #FIXME: SHould this just update to current position???
    #CURR_SERVO_POS = [0 for i in m_limits]
    #pass  #FIXME: not sure if this will work

def get_all_joints_pos(ave_count = POS_AVERAGE_COUNT, sleep_time = 0.05):
    global CURR_SERVO_POS, AVE_SERVO_POS
    AVE_SERVO_POS = CURR_SERVO_POS.copy()  # FIXME: will this work?
    time.sleep(sleep_time)
    for servo_num in range(len(m_limits)):
        curr_pos = servo.getPosition(servo_num)
        #ave_pos = (AVE_SERVO_POS[servo_num]*ave_count + curr_pos)/(ave_count+1)
        #print("curr_pos", curr_pos)
        CURR_SERVO_POS[servo_num] = curr_pos
        #AVE_SERVO_POS[servo_num] = ave_pos

rel_zero_pos = 18*[0]

def set_rel_zero_position():
    global pos_rel_zero
    for servo_num in range(len(m_limits)):
        curr_pos = servo.getPosition(servo_num)
        rel_zero_pos[servo_num] = curr_pos
        


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


def is_moving(thresh = MOVING_THRESH, sleep_time = 0.07):
    global CURR_SERVO_POS, AVE_SERVO_POS
    #update_ave_joints_pos()
    get_all_joints_pos(sleep_time=sleep_time)
    squared_sum = 0
    for servo_num in range(len(m_limits)):
        squared_sum += (CURR_SERVO_POS[servo_num] - AVE_SERVO_POS[servo_num])**2
    moving_amount = np.sqrt(squared_sum)
    if moving_amount >= thresh:
        return True
    else:
        return False 


# def wait_while_moving(thresh = MOVING_THRESH, delay = 0.01):
#     # moving = is_moving(thresh)
#     # while moving:
#     #     time.sleep(delay)
#     #     moving = is_moving(thresh)
#     pass


def wait_while_legs_moving(legs):
    servo_list = []
    for leg in legs:
        servo_list.append(3*leg+0)
        servo_list.append(3*leg+1)
        servo_list.append(3*leg+2)
    return servo.wait_while_moving(servo_list)      
    #return wait_while_any_moving()    

def wait_while_any_moving(): # Doesn't work
    while servo.getMovingState():
        pass
    return None


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
    #if wait: wait_while_moving()


def move_joint_angle(leg=0, joint=0, angle=0, wait=False, wait_till_execute = False): 
    reset_ave_servo_pos()
    servo_num = leg*3+joint
    
    joint_dir = m_limits[servo_num][2]
    joint_nom = int(m_limits[servo_num][3]*MLS)   # this corresponds to zero degrees

    angle_scale = m_limits[servo_num][6]  #476.25*4/90  # steps / degrees
    
    target = int(joint_nom + angle*angle_scale) #joint_dir*angle*SERVO_SCALE)

    if debug: print("leg, joint, angle, joint_dir, joint_nom, target:", leg, joint, angle, joint_dir, joint_nom, target)
    if debug: print("   limits: ", m_limits[servo_num][0]*MLS, m_limits[servo_num][1]*MLS)
    if not wait_till_execute:
        servo.setTarget(servo_num, target)
    else:
        NEXT_SERVO_POS[servo_num] = target
    #if not wait_till_execute and wait: servo.wait_while_moving(servo_num)


def execute_move_joint_angle(wait = False):
    if debug: print("Executing")
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


def legs_step_angles(legs, theta0, theta1, theta2, step_angle=30, wait_end=False, wait_each=True, delay = 0.3):
    reset_ave_servo_pos()
    if wait_each:
        for leg in legs:
            move_joint_angle(leg, 1, theta1+step_angle)
            #wait_while_legs_moving((leg,))
            time.sleep(delay)
            move_joint_angle(leg, 2, theta0, False)
            move_joint_angle(leg, 0, theta2, False)  # note thetaX and joint are opposite.  Just because.
            time.sleep(delay)
            #wait_while_legs_moving((leg,))
            move_joint_angle(leg, 1, theta1, False)
            time.sleep(delay)
            #wait_while_legs_moving((leg,))
    else:
        for leg in legs:
            move_joint_angle(leg, 1, theta1+step_angle)   
        wait_while_legs_moving(legs)
        for leg in legs:
            move_joint_angle(leg, 2, theta0, False)
            move_joint_angle(leg, 0, theta2, False)  # note thetaX and joint are opposite.  Just because.
            #time.sleep(0.25)
        wait_while_legs_moving(legs)
        for leg in legs:
            move_joint_angle(leg, 1, theta1, False)
        wait_while_legs_moving(legs)
        
    if wait_end: wait_while_legs_moving(legs)


def legs_move_relative(legs, delta_x, delta_y, delta_z, rel_zero = False, wait_end=False):
    global rel_zero_pos
    reset_ave_servo_pos()
    get_all_joints_pos()
    if rel_zero:
        curr_zero_pos = rel_zero_pos
    else:
        curr_zero_pos = CURR_SERVO_POS
    for leg in legs:  # FIXME: Move all at once
        theta2 = angle_from_steps(3*leg+0,curr_zero_pos[3*leg+0])
        theta1 = angle_from_steps(3*leg+1,curr_zero_pos[3*leg+1])
        theta0 = angle_from_steps(3*leg+2,curr_zero_pos[3*leg+2])
        x0, y0, z0 = forward_kinematics(theta0, theta1, theta2, L1, L2)
        if leg in left_side_legs:
            x, y, z = x0 - delta_x, y0 + delta_y, z0 + delta_z
        else:
            x, y, z = x0 + delta_x, y0 + delta_y, z0 + delta_z
        legs_move_xyz((leg,), x, y, z, False)
    if wait_end: wait_while_moving()


left_side_legs = (3,4,5)
    

def legs_step_relative(legs, delta_x, delta_y, delta_z, step_z_angle=35, rel_zero = False, wait_end=False, delay=0.15):
    global rel_zero_pos
    print("step relative: legs: ", legs)
    if legs == (): return
    #delay = 0.3
    reset_ave_servo_pos()
    get_all_joints_pos()
    if rel_zero:
        curr_zero_pos = rel_zero_pos
    else:
        curr_zero_pos = CURR_SERVO_POS
    for leg in legs:
        theta2 = angle_from_steps(3*leg+0,curr_zero_pos[3*leg+0])
        theta1 = angle_from_steps(3*leg+1,curr_zero_pos[3*leg+1])
        theta0 = angle_from_steps(3*leg+2,curr_zero_pos[3*leg+2])
        x0, y0, z0 = forward_kinematics(theta0, theta1, theta2, L1, L2)
        if leg in left_side_legs:
            x, y, z = x0 - delta_x, y0 + delta_y, z0 + delta_z
        else:
            x, y, z = x0 + delta_x, y0 + delta_y, z0 + delta_z
        theta0_new, theta1_new, theta2_new = inverse_kinematics(x,y,z, L1, L2)
        move_joint_angle(leg, 1, theta1+step_z_angle)
        time.sleep(delay)
        move_joint_angle(leg, 2, theta0_new)
        #time.sleep(delay)
        move_joint_angle(leg, 0, theta2_new)
        time.sleep(delay)
        move_joint_angle(leg, 1, theta1_new)
        time.sleep(delay)
        

def legs_turn_relative(legs, delta_angle, rel_zero = False, wait_end=False, delay=0.15):  # top joint only
    reset_ave_servo_pos()
    get_all_joints_pos()
    if rel_zero:
        curr_zero_pos = rel_zero_pos
    else:
        curr_zero_pos = CURR_SERVO_POS
    for leg in legs:
        theta2 = angle_from_steps(3*leg+0,curr_zero_pos[3*leg+0])
        theta1 = angle_from_steps(3*leg+1,curr_zero_pos[3*leg+1])
        theta0 = angle_from_steps(3*leg+2,curr_zero_pos[3*leg+2])
        if leg in left_side_legs:
            new_theta0 = theta0+delta_angle
        else:
            new_theta0 = theta0-delta_angle
        move_joint_angle(leg, 2, new_theta0)
        
def legs_lift_angle_relative(legs, delta_angle, rel_zero = False, wait_end=False, delay=0.15):  # mid joint only
    reset_ave_servo_pos()
    get_all_joints_pos()
    if rel_zero:
        curr_zero_pos = rel_zero_pos
    else:
        curr_zero_pos = CURR_SERVO_POS
    for leg in legs:
        #theta2 = angle_from_steps(3*leg+0,curr_zero_pos[3*leg+0])
        #theta1 = angle_from_steps(3*leg+1,curr_zero_pos[3*leg+1])
        #theta0 = angle_from_steps(3*leg+2,curr_zero_pos[3*leg+2])
        theta1 = curr_zero_pos[3*leg+1]  # FIXME: bypass calibration
        new_theta1 = theta1+delta_angle
        move_joint_angle(leg, 1, new_theta1)


############################################################################


############
### MAIN ###
############

pygame.init()
j = pygame.joystick.Joystick(0)   # Assume we only have 1
j.init()
clock = pygame.time.Clock()


print("Starting")

all_legs = (2,3,4,5,1,0)
actual_front_legs = (5,0)
actual_mid_legs = (4,1)
actual_back_legs = (3,2)


delay = 0.4
step_size = 30

direction = 0

# Set up legs

#legs_step_angles(actual_back_legs, 0, 10, -15)
#legs_step_angles(actual_front_legs, 45, 10, -15)
#legs_step_angles(actual_front_legs, 65, 10, -15) ########FIXME: Temp
#legs_step_angles(actual_mid_legs,   0, 0, -15, wait_each=False)
#legs_step_angles(actual_back_legs, -45, 10, -15)
#wait_while_legs_moving(all_legs)
reset_all_joints()
time.sleep(1.0)
get_all_joints_pos()
set_rel_zero_position()
time.sleep(1)

#assert False

#move_joint_angle(2,1, 35)
#legs_lift_angle_relative((2,), 30, False)
#time.sleep(1.0)
#move_joint_angle(2,1, 0)
#legs_lift_angle_relative((2,), 30, False)
#time.sleep(1.0)
#legs_turn_relative((2,), 30, False)
#legs_turn_relative((4,), 20, rel_zero=True)


def legs_lift(legs, steps):
    for leg in legs:
        move_joint_angle(leg, 1, steps)

def legs_turn(legs,steps):
    for leg in legs:
        move_joint_angle(leg, 2, steps)

#legs_turn((0,1,2, 3, 4, 5), -30)
#time.sleep(1)
#legs_turn((0,1,2, 3, 4,5), 0)
#time.sleep(1)


#legs_lift((0,1,2, 3, 4, 5), 45)
#time.sleep(1)
#legs_lift((0,1,2, 3, 4, 5), 0)
#time.sleep(1)

#legs_turn((0,5), 0)
#time.sleep(1)
#legs_lift((3,4,5), 0)


#assert False

def walk(direction=0, step_size=30, num_steps=1, delay=0.2):

    front_legs, mid_legs, back_legs, delta_x, delta_y = legs_directions[direction]
    print("front, mid, back, dx, dy: ", front_legs, mid_legs, back_legs, delta_x, delta_y)


    for count in range(num_steps):

        # 2
        legs_step_relative(front_legs, delta_x*step_size, 2*delta_y*step_size, 0, rel_zero=True)
        time.sleep(delay)

        #legs_step_relative(mid_legs, delta_x*step_size, delta_y*step_size, 0, rel_zero=True)
        #time.sleep(delay)

        # 3
        legs_move_relative(all_legs, -delta_x*step_size, -delta_y*step_size, 0)
        time.sleep(delay)
        
        # 4
        legs_step_relative(mid_legs, 0, 0, 0, rel_zero=True)
        time.sleep(delay)
        
        # 5
        legs_move_relative(all_legs, -delta_x*step_size, -delta_y*step_size, 0)
        time.sleep(delay)

        # 6
        legs_step_relative(mid_legs+back_legs, 0, 0, 0, rel_zero=True)
        time.sleep(delay)

# TEST WALKING in each direction
#walk(0, 15, 3, delay=0.25)
#time.sleep(0.5)
# walk(180, 30, 2, delay=0.25)
# time.sleep(0.5)
# walk(90, 30, 2, delay=0.25)
# time.sleep(0.5)
# walk(-90, 30, 2, delay=0.25)
# time.sleep(0.5)


#assert False

def crab_walk_turn(dist=30, steps=1, delay=0.35):

    turn_legs=(0,1,2,3,4,5)
    legs_left = (3,4,5)
    legs_right = (0,1,2)

    legs_turn(legs_left, -dist)
    legs_turn(legs_right, dist)
    time.sleep(delay)

    legs_odd = (1,3,5)
    legs_even = (0,2,4)

    for legs in (legs_odd, legs_even):

        legs_lift(legs, 30)
        time.sleep(delay)
        legs_turn(legs, 0)
        time.sleep(delay)
        legs_lift(legs, 0)
        time.sleep(delay)




def crab_walk_2(dir=0, dist=30, steps=1, delay=0.35):

    if (dir == 0) or (dir == 180):
        print("f/b")
        turn_legs_L = (5,3, 1)
        turn_legs_R = (0,2, 4)
    elif (dir == 90) or (dir == -90):
        print("l/r")
        turn_legs_L = (0,2,4)
        turn_legs_R = (1,5,3)
        
    
    angle = -dist

    for step in range(steps):

        # R
        legs_lift(turn_legs_L, 30)
        time.sleep(delay)
        legs_turn(turn_legs_R, -dist)
        time.sleep(delay)
        legs_lift(turn_legs_L, 0)
        time.sleep(delay)

        legs_lift(turn_legs_R, 30)
        time.sleep(delay)
        legs_turn(turn_legs_R, 0)
        time.sleep(delay)
        

        # L
        #legs_lift(other_legs_L, 30)
        #time.sleep(delay)
        legs_turn(turn_legs_L, -dist)
        time.sleep(delay)
        legs_lift(turn_legs_R, 0)
        time.sleep(delay)

        legs_lift(turn_legs_L, 30)
        time.sleep(delay)
        legs_turn(turn_legs_L, 0)
        time.sleep(delay)
        

        if step == (steps -1):
            legs_lift(turn_legs_L, 0)
            time.sleep(delay)
        
        #legs_step_relative(turn_legs, 0, 0, 0, 30, False)
        #time.sleep(delay)



#crab_walk_2(dist=-30, steps=6)




def start_ps4_ctrl(delay):

    ###### PS4 Remote Control ######

    turning = False
    walking = False
    muscle_up = False
    #busy = False

    while True:
        clock.tick(30) # Frame Rate = 30fps    

        events = pygame.event.get()
        for event in events:
            if event.type == pygame.JOYBUTTONDOWN:
                #print("Button Pressed: ", )
                print(event.dict, event.joy, ps4_button[event.button], 'pressed')
                #time.sleep(0.5)
            elif event.type == pygame.JOYBUTTONUP:
                print(event.dict, event.joy, ps4_button[event.button], 'released')
                #time.sleep(0.5)

        left_stick_x_axis = j.get_axis(0)
        left_stick_y_axis = -j.get_axis(1)
        right_stick_x_axis = j.get_axis(3)
        right_stick_y_axis = -j.get_axis(4)
        arm_left_axis = (j.get_axis(2) + 1) / 2   # change (-1,1) to (0,1)
        arm_right_axis = (j.get_axis(5) + 1) / 2 

        #print(f"{left_stick_x_axis:.2f},{left_stick_y_axis:.2f},"
        #      f"{right_stick_x_axis:.2f},{right_stick_y_axis:.2f}," 
        #      f"{arm_left_axis:.2f},{arm_right_axis:.2f},")

        deadzone = 0.25
        thresh1 = 0.9
        thresh2 = 0.99



        # Turning
        if not muscle_up and abs(right_stick_x_axis) > deadzone:
            print("turning")
            direction = right_stick_x_axis/abs(right_stick_x_axis)
            #legs_turn_relative(all_legs, 20*direction, True) #45*right_stick_x_axis, rel_zero=True)
            crab_walk_turn(20*direction)
            #time.sleep(delay)
            #legs_step_relative(all_legs, 0, 0, 0, rel_zero = True)
            turning = True
            time.sleep(delay)
        elif turning and abs(right_stick_y_axis) <= deadzone:
            #crab_walk_turn(0)
            #legs_turn_relative(all_legs, 0, rel_zero=True)
            #time.sleep(delay)
            turning = False

        # Muscle-up
        if not turning and abs(right_stick_y_axis) > deadzone:
            #print("muscle_up", right_stick_y_axis)
            #legs_move_relative(all_legs, 0, 0, -50*right_stick_y_axis, rel_zero=True)
            legs_lift(all_legs, -30*right_stick_y_axis)
            muscle_up = True
            time.sleep(delay)
        elif muscle_up and abs(right_stick_y_axis) <= deadzone:
            legs_lift(all_legs, 0)
            muscle_up = False
        

        # Walking

        if not muscle_up and not turning and abs(left_stick_y_axis) > deadzone:
            print("walking fwd/back")
            if left_stick_y_axis > 0:
                crab_walk_2(0, 30, 1)
            else:
                crab_walk_2(0, -30, 1)
            
            walking = True
            time.sleep(delay)
        elif not muscle_up and not turning and abs(left_stick_x_axis) > deadzone:
            print("walking left/right")
            if left_stick_x_axis > 0:
                crab_walk_2(90, 30, 1)
            else:
                crab_walk_2(90, 30, 1)
            walking = True
            #     time.sleep(delay)
        elif walking and (abs(left_stick_y_axis) <= deadzone) and (abs(left_stick_x_axis) <= deadzone):
            walking = False


start_ps4_ctrl(delay)










# Cleanup

servo.close()