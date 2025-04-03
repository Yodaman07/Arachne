# Arachne Control and Walking

# ToDo:
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



#import maestro
import maestro_extended
import time
import numpy as np
import sys

debug = False


# Define Motor limits and characteristics

m_limits = [
    
    # Right Side

    # FIXME: Not using min/max right now.  Do I need to?
    
    [608, 1920, 1, 1122.25, 90, 1773.5, 999],  # min, max, direction, nominal [0 degrees], setting [calibrated], calibrated angle. slope [calc'd later]
    [800, 1856, -1, 1277.5, -45, 1512.50, 999],
    [1328, 2000, -1, 1815.25, 45, 1458.25, 999],
    
    [64, 1136, -1, 848.25, 38, 646.25, 999],
    [800, 1840, -1, 1405.75, -45, 1643.0, 999],
    [1568, 2496, 1, 1901.25, 45, 2189.5, 999 ],

    [800, 2096, 1, 1250.0, 90, 1976.25, 999],
    [1008, 2096, -1, 1441.75, -45, 1668.0, 999],
    [1008, 2048, -1, 1119.0, -45, 1618.75, 999],
    
    # Left Side
    
    [704, 1904, -1, 1468.25, 90, 832.0, 999],  #  9
    [496, 1600, 1, 1195.25, -45, 929.25, 999],   # 10
    [1200, 2352, 1, 2305.00, -90, 1390.0, 999 ],   # 11
    
    [608, 1904, -1, 1435.25, 90, 784.0, 999],
    [752, 1600, 1, 1214.25, -45, 891.75, 999],
    [1008, 2000, 1, 1556.75, 45, 1937.25, 999],
    
    [608, 1904, -1, 1500.0, 90, 803.0, 999],
    [752, 1504, 1, 1147.0, -45, 799.25, 999],
    [688, 1296, 1, 780.75, 45, 1139.5, 999],
    ]


L1, L2 = 85, 123  # mm  #FIXME: Measure better



############# Other Definitions ################

ACCEL = 30   # Global acceleration setting for all servos
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
      0: [(5,0), (4,1), (3,2), 0, 1],
     45: [(0,1), (5,2), (4,3), 0.7, 0.7],
     90: [(0,1,2), (), (3,4,5), 1, 0],
    135: [(1,2), (3,0), (4,5), 0.7, -0.7],
    180: [(2,3), (4,1), (5,0), 0, -1],
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
    if wait: wait_while_moving()


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
    if not wait_till_execute and wait: servo.wait_while_moving(servo_num)


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


def legs_step_angles(legs, theta0, theta1, theta2, step_angle=30, wait_end=False, wait_each=True):
    reset_ave_servo_pos()
    if wait_each:
        for leg in legs:
            move_joint_angle(leg, 1, theta1+step_angle)
            wait_while_legs_moving((leg,))
            move_joint_angle(leg, 2, theta0, False)
            move_joint_angle(leg, 0, theta2, False)  # note thetaX and joint are opposite.  Just because.
            wait_while_legs_moving((leg,))
            move_joint_angle(leg, 1, theta1, False)
            wait_while_legs_moving((leg,))
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


def legs_step_relative(legs, delta_x, delta_y, delta_z, step_z=75, wait_end=False):
    print("step relative: legs: ", legs)
    if legs == (): return
    delay = 0.3
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
        #wait_while_legs_moving((leg,))
        time.sleep(delay)
        legs_move_xyz((leg,), x, y, z0+step_z, False)
        #wait_while_legs_moving((leg,))
        time.sleep(delay)
        legs_move_xyz((leg,), x, y, z, False)
        #wait_while_legs_moving((leg,))
        time.sleep(delay)
        

    
############################################################################




print("Starting")

# Get to default walking position
#reset_all_joints()
# print(wait_while_legs_moving((0,1,2,3,4,5)))
# # wait_while_moving()
# #time.sleep(1)
# legs_move_angles((0,5,), 60, 0, -20, False)
# legs_move_angles((2,3,), -60, 0, -20, False)
# legs_move_angles((1,4,), 0, 0, -20, False)
# ret = wait_while_legs_moving((0,1,2,3,4,5))
# print ("Ret:", ret)


all_legs = (0,1,2,4,5)
all_legs = (2,3,4,5,1,0
            )
actual_front_legs = (5,0)
actual_mid_legs = (4,1)
actual_back_legs = (3,2)

#move_joint_angle(3, 2, -90)
#servo.setTarget(11, 7400)  #4800, 9408

# legs_step_angles(back_legs, -45 , 10, -25)
# time.sleep(1)
# legs_move_relative(back_legs, 0, -80, 0)
# sys.exit()


#legs_step_angles((3,), 30, 40, 0)

# # Walking not too bad
# for count in range(3):

#     legs_step_angles((5,0,), 55, 10, -5)
#     legs_step_angles((4,1), 25, 10, -25, wait_each=False)
#     #legs_step_angles((3,2,), -15, 10, -25)
#     legs_step_angles((2,), -40, 10, -25)
    
#     wait_while_legs_moving(all_legs)
#     #time.sleep(1)

#     legs_move_relative((0,1,2,4,5), 0, -60, 0, False)  # was 0, -70, 0
#     wait_while_legs_moving(all_legs)
#     time.sleep(0.5)
    

#legs_move_angles(back_legs, -45 , 10, -25)


delay = 0.3
step_size = 30

direction = 0
#FIXME: 90 degree direction, weird double move



front_legs, mid_legs, back_legs, delta_x, delta_y = legs_directions[direction]
print("front, mid, back, dx, dy: ", front_legs, mid_legs, back_legs, delta_x, delta_y)
#sys.exit()

for count in range(3):

    print("*** 1 ***")
    legs_step_angles(actual_front_legs, 45, 10, -15)
    legs_step_angles(actual_mid_legs,   10, 10, -15, wait_each=False)
    legs_step_angles(actual_back_legs, -45, 10, -15)
    #wait_while_legs_moving(all_legs)
    time.sleep(delay)

    #input("Press Enter to continue...")
    print("*** 2 ***")
    #legs_step_angles(front_legs, 65, 10, -5)
    legs_step_relative(front_legs, delta_x*step_size, 2*delta_y*step_size, 0)
    time.sleep(delay)
    #wait_while_legs_moving(all_legs)
    
    #input("Press Enter to continue...")
    print("*** 3 ***")
    legs_move_relative(all_legs, delta_x*step_size, -delta_y*step_size, 0)
    time.sleep(delay)
    #wait_while_legs_moving(all_legs)
    
    #input("Press Enter to continue...")
    print("*** 4 ***")
    legs_step_relative(mid_legs, delta_x*step_size, delta_y*step_size, 0)
    time.sleep(delay)
    #wait_while_legs_moving(all_legs)
    
    #input("Press Enter to continue...")
    print("*** 5 ***")
    legs_move_relative(all_legs, delta_x*step_size, -delta_y*step_size, 0)
    time.sleep(delay)
    # #wait_while_legs_moving(all_legs)
    
    
legs_step_angles(actual_front_legs, 45, 10, -15)
legs_step_angles(actual_mid_legs,   10, 10, -15, wait_each=False)
legs_step_angles(actual_back_legs, -45, 10, -15)
#wait_while_legs_moving(all_legs)
time.sleep(delay)


# Cleanup

servo.close()