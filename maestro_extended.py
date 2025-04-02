import time
import maestro

class ExtendedController(maestro.Controller):
    def __init__(self, *args, stall_timeout=1.0, check_interval=0.1, **kwargs):
        """
        Initialize the extended controller.
        :param stall_timeout: Time in seconds before a servo is considered stalled.
        :param check_interval: Time in seconds between position checks.
        """
        super().__init__(*args, **kwargs)
        self.stall_timeout = stall_timeout
        self.check_interval = check_interval
        self.servo_state = {}  # Dictionary to track each servo's state

    def getTarget(self, channel):
        """
        Retrieves the last commanded target position for a given servo channel.
        :param channel: Servo channel to check.
        :return: The last target position set for the servo.
        """
        return self.Targets[channel]  # Targets is a class variable in maestro.py

    def isStalled(self, channel):
        """
        Check if a servo is stalled by comparing the current and last position over time.
        :param channel: Servo channel to check.
        :return: True if stalled, False otherwise.
        """
        current_pos = self.getPosition(channel)
        target_pos = self.getTarget(channel)

        if channel not in self.servo_state:
            # Initialize tracking for this servo
            self.servo_state[channel] = {
                "last_pos": current_pos,
                "last_time": time.time(),
            }
            return False

        state = self.servo_state[channel]

        if current_pos == state["last_pos"] and current_pos != target_pos:
            # If position hasn't changed and is not at the target, check the time
            if time.time() - state["last_time"] > self.stall_timeout:
                return True  # Stalled
        else:
            # Update tracking if the servo is moving
            state["last_pos"] = current_pos
            state["last_time"] = time.time()

        return False

    def wait_while_moving(self, channels):
        """
        Waits while the specified servos are moving. Returns 'Stalled' if any servo is stuck.
        :param channels: List of servo channels to check.
        :return: 'Stalled' if any servo is stalled, otherwise None.
        """
        if isinstance(channels, int):  # Allow single channel input
            channels = [channels]

        while any(self.isMoving(ch) for ch in channels):
            for ch in channels:
                if self.isStalled(ch):
                    return "Stalled"
            time.sleep(self.check_interval)
        
        return None  # All servos reached their targets
