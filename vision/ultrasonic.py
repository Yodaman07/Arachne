from gpiozero import DistanceSensor


#https://gpiozero.readthedocs.io/
def testDistance():
    ultrasonic = DistanceSensor(echo=0, trigger=0)  # SET PINS
    print(ultrasonic)
