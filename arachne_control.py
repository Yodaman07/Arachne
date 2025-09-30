# Arachne Control and Walking

# ToDo:
# * Servo 10 (in Leg 3) needs to be fine-tuned for ranges
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

# FIXED: the new motor is 995 instead of 996.  Doesn't move smoothly.  But only in Joystick mode???

import pygame
import cv2 as cv
import threading

import maestro_extended
import time
import networking.client as Client
import regex as re


class ArachneController:
    def __init__(self, debug):
        print("Starting up Arachne")
        # Set up legs
        self.debug = debug

        pygame.init()

        self.j = pygame.joystick.Joystick(0)  # Assume we only have 1
        self.clock = pygame.time.Clock()
        self.j.init()
        if self.j.get_init():
            print("Joystick Ready!")

        self.autonomous = False  # teleop mode for default
        self.pause_event = threading.Event()

        # Initializing all leg ids
        self.all_legs = (2, 3, 4, 5, 1, 0)
        self.actual_front_legs = (5, 0)
        self.actual_mid_legs = (4, 1)
        self.actual_back_legs = (3, 2)

        # Define Motor limits and characteristics
        self.m_limits = [

            # Right Side

            # FIXME: Not using min/max right now.  Do I need to?

            [608, 1920, 1, 831.25, 90, 1773.5, 999],
            # min, max, direction, nominal [0 degrees], setting [calibrated], calibrated angle. slope [calc'd later]
            [544, 1152, -1, 863, 45, 54, 999],  # 1
            [1248, 1840, -1, 1492, -45, 1840, 999],  # 2

            [64, 1136, -1, 1040, 38, 646.25, 999],  # 3
            [608, 1328, -1, 1020, 45, 608.0, 999],  # 4
            [1568, 2128, 1, 1901.25, 45, 2073, 999],  # 5

            [800, 2096, 1, 1250.0, 90, 1976.25, 999],  # 6
            [608, 1344, -1, 1019, 45, 608.0, 999],  # 7
            [736, 1632, -1, 1400, 45, 736, 999],  # 8   # was x, x, 1270

            # Left Side

            [704, 1904, -1, 1468.25, 90, 832.0, 999],  # 9
            [496, 1216, 1, 807, 45, 1216, 999],  # 10. #FIXME: replaced Servo, needs new Cal
            [1248, 2080, 1, 1450.00, 45, 2031, 999],  # 11   # was x, x, 1550

            [608, 1760, -1, 1615, 90, 784.0, 999],  # 12
            [1328, 2000, 1, 1556, 45, 2000, 999],  # 13
            [1008, 1824, 1, 1556.75, 45, 1790, 999],  # 14

            [608, 1904, -1, 1734.0, 90, 803.0, 999],  # 15
            [1232, 1952, 1, 1504.0, 45, 1952, 999],  # 16
            [848, 1312, 1, 1156, -45, 848, 999],  # 17
        ]

        self.L1, self.L2 = 85, 123  # mm  #FIXME: Measure better

        ############# Other Definitions ################

        self.ACCEL = 20  # Global acceleration setting for all servos
        self.SERVO_SPEED = 200
        self.POS_AVERAGE_COUNT = 3  # Number of position readings to average together
        self.MLS = 4  # m_limits_scale - multiply all of the above for what should actually be written to the Maestro (not sure why)
        self.MOVING_THRESH = 10  # How far away is "close-enough" to be still-moving

        self.CURR_SERVO_POS = [0 for i in self.m_limits]
        self.AVE_SERVO_POS = [servo_setting[3] * self.MLS for servo_setting in
                              self.m_limits]  # Start at nominal for each
        self.NEXT_SERVO_POS = [0 for i in self.m_limits]

        # SERVO_SCALE = 476.25*MLS/90.0   # steps / degrees

        # Define legs for different directions
        self.legs_directions = {
            0: [(5, 0), (4, 1), (3, 2), 0, 1],  # (front legs), (mid legs), (back legs), step x_scale, step y_scale]
            45: [(0, 1), (5, 2), (4, 3), 0.7, 0.7],
            90: [(1,), (0, 2), (3, 4, 5), 1, 0],
            135: [(1, 2), (3, 0), (4, 5), 0.7, -0.7],
            180: [(2, 3), (4, 1), (5, 0), 0, -1],
            -45: [(4, 5), (3, 0), (2, 1), -0.7, -0.7],
            -90: [(4,), (3, 5), (2, 1, 0), -1, 0],
            -135: [(3, 4), (2, 5), (1, 0), -0.7, +0.7],
        }

        # PS4 Buttons
        self.ps4_button = {
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
        self.servo = maestro_extended.ExtendedController(device=12, stall_timeout=0.5, check_interval=0.05)
        self.setServoLimits()

        self.reset_all_joints()
        time.sleep(1.0)
        self.get_all_joints_pos()
        time.sleep(1)

    ##### Config Functions ##########

    def reset_ave_servo_pos(self):
        self.AVE_SERVO_POS = [0 for i in self.m_limits]  # FIXME: SHould this just update to current position???
        # CURR_SERVO_POS = [0 for i in m_limits]
        # pass  #FIXME: not sure if this will work

    def get_all_joints_pos(self, sleep_time=0.05):

        self.AVE_SERVO_POS = self.CURR_SERVO_POS.copy()  # FIXME: will this work?
        time.sleep(sleep_time)
        for servo_num in range(len(self.m_limits)):
            curr_pos = self.servo.getPosition(servo_num)
            # ave_pos = (AVE_SERVO_POS[servo_num]*ave_count + curr_pos)/(ave_count+1)
            # print("curr_pos", curr_pos)
            self.CURR_SERVO_POS[servo_num] = curr_pos
            # AVE_SERVO_POS[servo_num] = ave_pos

    def setServoLimits(self):
        # Set software limits to be the same as the hardware limits, calculate slopes
        for i in range(len(self.m_limits)):
            self.servo.setRange(i, (self.m_limits[i][0]) * self.MLS, (self.m_limits[i][1]) * self.MLS)

            self.servo.setAccel(i, self.ACCEL)
            self.servo.setSpeed(i, self.SERVO_SPEED)
            cal1_angle = 0  # angle of first cal point
            cal1_setting = self.m_limits[i][3]
            cal2_angle = self.m_limits[i][4]
            cal2_setting = self.m_limits[i][5]
            slope = (cal2_setting - cal1_setting) / (cal2_angle - cal1_angle)

            self.m_limits[i][6] = slope * self.MLS

            print(self.m_limits[i])

    def reset_all_joints(self):
        self.reset_ave_servo_pos()
        for servo_num in range(len(self.m_limits)):
            joint_nom = int(self.m_limits[servo_num][3] * self.MLS)
            self.servo.setTarget(servo_num, joint_nom)

    # MAIN METHOD
    def start_ps4_ctrl(self, socket):
        ###### PS4 Remote Control ######
        # consider using a dedicated library for ps4 controlling with pyPS4Controller
        # pip install pyPS4Controller

        turning = False
        walking = False
        muscle_up = False
        # busy = False

        data = {"points": []}
        autonomousThread = threading.Thread(target=self.controller_mixin, args=data)
        autonomousThread.start()
        self.pause_event.clear()  # by default start thread and then pause it
        cap = cv.VideoCapture(0)

        while True:
            self.clock.tick(10)  # Frame Rate = 10fps

            events = pygame.event.get()
            for event in events:
                if event.type == pygame.JOYBUTTONDOWN:
                    # print("Button Pressed: ", )
                    print(event.dict, event.joy, self.ps4_button[event.button], 'pressed')
                    # time.sleep(0.5)
                elif event.type == pygame.JOYBUTTONUP:
                    if event.dict['button'] == 2:  # triangle
                        self.autonomous = not self.autonomous
                        if not self.autonomous:
                            self.pause_event.clear()  # pauses auto
                        else:
                            self.pause_event.set()  # starts autonomous

                        status = "on" if self.autonomous else "off"
                        print(f"Toggling Autonomous, status is now {status}")
                    print(event.dict, event.joy, self.ps4_button[event.button], 'released')
                    # time.sleep(0.5)
            left_stick_x_axis = self.j.get_axis(0)
            left_stick_y_axis = -self.j.get_axis(1)
            right_stick_x_axis = self.j.get_axis(3)
            right_stick_y_axis = -self.j.get_axis(4)

            exit = self.j.get_button(1)  # Circle

            arm_left_axis = (self.j.get_axis(2) + 1) / 2  # change (-1,1) to (0,1)
            arm_right_axis = (self.j.get_axis(5) + 1) / 2

            # print(f"{left_stick_x_axis:.2f},{left_stick_y_axis:.2f},"
            #      f"{right_stick_x_axis:.2f},{right_stick_y_axis:.2f},"
            #      f"{arm_left_axis:.2f},{arm_right_axis:.2f},")

            deadzone = 0.25
            thresh1 = 0.9
            thresh2 = 0.99

            if exit:
                print("Exiting")
                self.servo.close()
                break

            if not self.autonomous:
                # Turning
                if not muscle_up and abs(right_stick_x_axis) > deadzone:
                    print("turning")
                    direction = right_stick_x_axis / abs(right_stick_x_axis)
                    self.crab_walk_turn(30 * direction)
                    turning = True
                elif turning and abs(right_stick_y_axis) <= deadzone:
                    turning = False

                # Muscle-up
                if not turning and abs(right_stick_y_axis) > deadzone:
                    # print("muscle_up", right_stick_y_axis)
                    self.legs_lift(self.all_legs, -30 * right_stick_y_axis)
                    muscle_up = True
                    # time.sleep(delay)
                elif muscle_up and abs(right_stick_y_axis) <= deadzone:
                    self.legs_lift(self.all_legs, 0)
                    muscle_up = False

                # Walking
                if not muscle_up and not turning and abs(left_stick_y_axis) > deadzone:
                    print("walking fwd/back")
                    if left_stick_y_axis > 0:
                        self.crab_walk_2(0, 30, 1)
                    else:
                        self.crab_walk_2(0, -30, 1)

                    walking = True
                elif not muscle_up and not turning and abs(left_stick_x_axis) > deadzone:
                    print("walking left/right")
                    if left_stick_x_axis > 0:
                        self.crab_walk_2(90, 30, 1)
                    else:
                        self.crab_walk_2(90, 30, 1)
                    walking = True
                elif walking and (abs(left_stick_y_axis) <= deadzone) and (abs(left_stick_x_axis) <= deadzone):
                    walking = False
            else:
                Client.client_tick(cap, socket, data)
        # Cleanup

    # Movement Functions
    def legs_lift(self, legs, steps):
        for leg in legs:
            self.move_joint_angle(leg, 1, steps)

    def legs_turn(self, legs, steps):
        for leg in legs:
            self.move_joint_angle(leg, 2, steps)

    def move_joint_angle(self, leg=0, joint=0, angle=0, wait=False, wait_till_execute=False):
        self.reset_ave_servo_pos()
        servo_num = leg * 3 + joint

        joint_dir = self.m_limits[servo_num][2]
        joint_nom = int(self.m_limits[servo_num][3] * self.MLS)  # this corresponds to zero degrees

        angle_scale = self.m_limits[servo_num][6]  # 476.25*4/90  # steps / degrees

        target = int(joint_nom + angle * angle_scale)  # joint_dir*angle*SERVO_SCALE)

        if self.debug: print("leg, joint, angle, joint_dir, joint_nom, target:", leg, joint, angle, joint_dir,
                             joint_nom,
                             target)
        if self.debug: print("   limits: ", self.m_limits[servo_num][0] * self.MLS,
                             self.m_limits[servo_num][1] * self.MLS)
        if not wait_till_execute:
            self.servo.setTarget(servo_num, target)
        else:
            self.NEXT_SERVO_POS[servo_num] = target
        # if not wait_till_execute and wait: servo.wait_while_moving(servo_num)

    def crab_walk_turn(self, dist=30, steps=1, delay=0.3):
        turn_legs = (0, 1, 2, 3, 4, 5)
        legs_left = (3, 4, 5)
        legs_right = (0, 1, 2)

        self.legs_turn(legs_left, -dist)
        self.legs_turn(legs_right, dist)
        time.sleep(delay)

        legs_odd = (1, 3, 5)
        legs_even = (0, 2, 4)

        for legs in (legs_odd, legs_even):
            self.legs_lift(legs, 30)
            time.sleep(delay)
            self.legs_turn(legs, 0)
            time.sleep(delay)
            self.legs_lift(legs, 0)
            time.sleep(delay)

    def crab_walk_2(self, dir=0, dist=30, steps=1, delay=0.3):
        # FIXME: Sideways needs different leg directions compared with fw/back

        if (dir == 0) or (dir == 180):
            print("f/b")
            turn_legs_L = (5, 3, 1)
            turn_legs_R = (0, 2, 4)
        elif (dir == 90) or (dir == -90):
            print("l/r")
            turn_legs_L = (0, 2, 4)
            turn_legs_R = (1, 5, 3)

        if (dir == 0) and (dist < 0):  # FIXME: why does backwards step back so big???
            dist = dist / 2

        for step in range(steps):

            # R
            self.legs_lift(turn_legs_L, 30)
            time.sleep(delay)
            self.legs_turn(turn_legs_R, -dist)
            time.sleep(delay)
            self.legs_lift(turn_legs_L, 0)
            time.sleep(delay)

            self.legs_lift(turn_legs_R, 30)
            time.sleep(delay)
            self.legs_turn(turn_legs_R, 0)
            time.sleep(delay)

            # L
            # legs_lift(other_legs_L, 30)
            # time.sleep(delay)
            self.legs_turn(turn_legs_L, -dist)
            time.sleep(delay)
            self.legs_lift(turn_legs_R, 0)
            time.sleep(delay)

            self.legs_lift(turn_legs_L, 30)
            time.sleep(delay)
            self.legs_turn(turn_legs_L, 0)
            time.sleep(delay)

            if step == (steps - 1):
                self.legs_lift(turn_legs_L, 0)
                time.sleep(delay)

            # legs_step_relative(turn_legs, 0, 0, 0, 30, False)
            # time.sleep(delay)

    def controller_mixin(self, pt):  # points is a String to be parsed
        print("Init Thread")
        # Pt Key:
        # Formatted like {"points": [[x,y],[x,y],[x,y]]}, Options are: "NA", "[_,_]" <- actual point
        while True:
            self.pause_event.wait()

            if pt[0] == "NA":  # scanning for an object
                print("TURN")
                self.crab_walk_turn(10)
            else:
                print(pt[0])  # if multiple points try to walk in between them, only sending one for now though
                print("CRAB WALK")
                x, y = pt[0][0][0], pt[0][0][1]
                if x < (320-20): # width is 640, # turn right
                    self.crab_walk_turn(10)
                elif x > (320+20):  # turn left
                    self.crab_walk_turn(-10)

                self.crab_walk_2(0, 30, 1)
