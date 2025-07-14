from arachne_control import ArachneController, Vision
import cv2 as cv

if __name__ == "__main__":
    # controller = ArachneController(debug=False)
    v = Vision()
    res = True
    while res:
        res = v.tick()
    # controller.start_ps4_ctrl()  # could consider a different method of control?

# Brainstorming possible project goals:
# - Developing a new method of control allowing for individual limb control like with an arcade button controller setup
#           - Are we letting the kids control arachne as well?
# - Implementing a vision system w a camera or ultrasonic sensor to prevent arachne from bumping into people and objects
#           - Autonomous pathfinding where Arachne can detect places it can move to and walk around, demoing itself. Similar to how phyz will look at people when idle and move his head around, arachne could move around within a set space and slightly branch out based off what space is deemed "available"
# - IMU to measure how off balance arachne is and to keep him upright and prevent tipping

