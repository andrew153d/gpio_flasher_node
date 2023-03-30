import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time

GPIO_PIN = 19
FLASH_INTERVAL = 1

last_status = False
flash_timer = None

def status_callback(msg):
    global last_status, flash_timer
    current_status = msg.data
    if current_status:
        flash_timer = rospy.Timer(rospy.Duration(FLASH_INTERVAL), flash_callback)
    else:
        if flash_timer:
            flash_timer.shutdown()
        GPIO.output(GPIO_PIN, GPIO.HIGH)
    last_status = current_status

def flash_callback(event):
    GPIO.output(GPIO_PIN, not GPIO.input(GPIO_PIN))

def autonomous_status():
    rospy.init_node('autonomous_status', anonymous=True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_PIN, GPIO.OUT)
    rospy.Subscriber('status/isAutonomous', Bool, status_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        autonomous_status()
    except rospy.ROSInterruptException:
        pass