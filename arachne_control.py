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

# FIXME: the new motor is 995 instead of 996.  Doesn't move smoothly.  But only in Joystick mode???

import pygame
from pygame.joystick import JoystickType
from ai_edge_litert.interpreter import Interpreter
import cv2 as cv

import maestro_extended
import time
import numpy as np


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
        self.autonomous = True  # teleop mode for default
        self.vision = Vision(self)

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
            [1248, 1968, 1, 1580, 45, 1968, 999],  # 10
            [1248, 2080, 1, 1450.00, 45, 2031, 999],  # 11   # was x, x, 1550

            [608, 1760, -1, 1615, 90, 784.0, 999],  # 12
            [1328, 2000, 1, 1556, 45, 2000, 999],  # 13
            [1008, 1824, 1, 1556.75, 45, 1790, 999],  # 14

            [608, 1904, -1, 1734.0, 90, 803.0, 999],  # 15
            [1232, 1952, 1, 1504.0, 45, 1952, 999],  # 16
            [848, 1312, 1, 1156, -45, 848, 999],  # 17
        ]
        self.m_limits_2 = [

            # Right Side

            # FIXME: Not using min/max right now.  Do I need to?

            [608, 1920, 1, 831, 90, 1773.5, 999],
            # min, max, direction, nominal [0 degrees], setting [calibrated], calibrated angle. slope [calc'd later]
            [704, 1392, -1, 964, -45, 1392, 999],  # 1
            [1248, 1840, 1, 1380, -45, 1840, 999],  # 2

            [64, 1136, -1, 1040, 38, 646.25, 999],  # 3
            [800, 1840, -1, 1160, -45, 1643.0, 999],  # 4
            [1568, 2496, 1, 1901.25, 45, 2189.5, 999],  # 5

            [800, 2096, 1, 1250.0, 90, 1976.25, 999],  # 6
            [800, 1504, -1, 1022, -45, 1668.0, 999],  # 7
            [736, 1632, -1, 1320.0, -90, 1931.75, 999],  # 8

            # Left Side

            [704, 1904, -1, 1468.25, 90, 832.0, 999],  # 9
            [1088, 1840, 1, 1600, -45, 929.25, 999],  # 10
            [1248, 2080, 1, 1470.00, -90, 974.75, 999],  # 11

            [608, 1760, -1, 1615, 90, 784.0, 999],  # 12
            [752, 1600, 1, 1414.25, -45, 891.75, 999],  # 13
            [1008, 2000, 1, 1556.75, 45, 1937.25, 999],  # 14

            [608, 1904, -1, 1734.0, 90, 803.0, 999],  # 15
            [1040, 1808, 1, 1427.0, -45, 1044, 999],  # 16
            [848, 1504, 1, 1296, -45, 848, 999],  # 17
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
    def start_ps4_ctrl(self):
        ###### PS4 Remote Control ######
        # consider using a dedicated library for ps4 controlling with pyPS4Controller
        # pip install pyPS4Controller

        turning = False
        walking = False
        muscle_up = False
        # busy = False

        res = True
        while res:
            self.clock.tick(30)  # Frame Rate = 30fps

            events = pygame.event.get()
            for event in events:
                if event.type == pygame.JOYBUTTONDOWN:
                    # print("Button Pressed: ", )
                    print(event.dict, event.joy, self.ps4_button[event.button], 'pressed')
                    # time.sleep(0.5)
                elif event.type == pygame.JOYBUTTONUP:
                    if event.dict['button'] == 2:
                        self.autonomous = not self.autonomous
                        print(f"Toggling Autonomous status is now {self.autonomous}")
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

            else:  # autonomous activated
                res = self.vision.tick()

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


# File courtesy of messing around with chatgpt
class Vision:
    def __init__(self, ac: ArachneController):
        self.ac = ac
        # Paths
        self.MODEL_PATH = "vision/model/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.tflite"
        self.LABEL_PATH = "vision/model/labelmap.txt"  # COCO Dataset

        # Load labels
        with open(self.LABEL_PATH, 'r') as f:
            self.labels = [line.strip() for line in f.readlines()]

        # Load model
        self.interpreter = Interpreter(model_path=self.MODEL_PATH)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        input_shape = self.input_details[0]['shape']
        self.height = input_shape[1]
        self.width = input_shape[2]
        self.floating_model = self.input_details[0]['dtype'] == np.float32

        self.new_frame_time = 0
        self.prev_frame_time = 0
        self.cap = cv.VideoCapture(0)

    def tick(self) -> bool:  # 1 frame

        retrieved, frame = self.cap.read()

        if not retrieved:
            print("Stream has likely ended")
            return False

        # cv.imshow("stream", frame)
        # https://stackoverflow.com/questions/5217519/what-does-opencvs-cvwaitkey-function-do <-- how waitKey works

        image_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        image_resized = cv.resize(image_rgb, (self.width, self.height))
        input_data = np.expand_dims(image_resized, axis=0)

        if self.floating_model:
            input_data = (np.float32(input_data) - 127.5) / 127.5
        else:
            input_data = np.uint8(input_data)

        # Run inference
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        # Get outputs
        boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
        scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]
        num = self.interpreter.get_tensor(self.output_details[3]['index'])[0]

        # Draw detections
        imH, imW, _ = frame.shape
        for i in range(int(num)):
            if scores[i] > 0.5:
                ymin = int(max(1, boxes[i][0] * imH))
                xmin = int(max(1, boxes[i][1] * imW))
                ymax = int(min(imH, boxes[i][2] * imH))
                xmax = int(min(imW, boxes[i][3] * imW))

                class_id = int(classes[i])
                label = self.labels[class_id] if class_id < len(self.labels) else "N/A"
                confidence = int(scores[i] * 100)

                center = (xmin + int(abs(xmin - xmax) / 2), ymin + int(abs(ymax - ymin) / 2))
                cv.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv.putText(frame, f"{label} ({confidence}%)", (xmin, ymin - 10),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # Show output

        self.new_frame_time = time.time()
        fps = 1 / (self.new_frame_time - self.prev_frame_time)
        self.prev_frame_time = self.new_frame_time  # Get fps

        # Display FPS on the frame (optional)
        # cv.putText(frame, f"FPS: {fps}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # cv.imshow("Object Detection", frame)

        if num <= 0:  # scanning
            self.ac.crab_walk_turn(10)  # keep turning until you detect something
        # # else: # start walking forward
        # #     cv.line(frame, center, center, (0, 0, 0), 20) # if you start walking forward, put a dot on the center of the detected object
        # #     # fine adjustment
        # #     actual_w = cv.CAP_PROP_FRAME_WIDTH/2
        # #     if 0 <= (center[0]-actual_w) <= 10:
        # #         arachne_control.crab_walk_turn(30) # move left
        # #     elif 0<= (actual_w - center[0]) <= 10:
        # #         arachne_control.crab_walk_turn(-30) # move right

        # #     arachne_control.crab_walk_2(0, 30, 1)

        if cv.waitKey(1) == ord("q"):  # ESC to quit
            return False

        return True

        # # if joystick.get_button(1): # exit program if the toggle is pressed
        # #     self.cap.release()
        # #     cv.destroyAllWindows()
