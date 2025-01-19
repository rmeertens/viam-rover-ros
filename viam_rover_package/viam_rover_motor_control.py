import rclpy
from rclpy.node import Node
import time
import RPi.GPIO as GPIO

from std_msgs.msg import String

RIGHT_MOTOR_ENABLE_ON=15
RIGHT_MOTOR_A = 13
RIGHT_MOTOR_B = 11


LEFT_MOTOR_ENABLE_ON=15
LEFT_MOTOR_A = 13
LEFT_MOTOR_B = 11


class TwistCallback(Node):

    def __init__(self):
        super().__init__('viam_twist_callback')
        self.publisher_ = self.create_subscription(String, 'cmd_vel', self.listener_callback, 10)
        print("minimal subscriber init")
        self.i = 0

    def listener_callback(self, msg):
        print("hello world", msg.data)
        self.i += 1


def main(args=None):
    # Set up ROS
    rclpy.init(args=args)

    # Set up the GPIO nodes for odometry
    GPIO.setmode(GPIO.BOARD)
    

    GPIO.setup(RIGHT_MOTOR_ENABLE_ON, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_A, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_B, GPIO.OUT)
    # p = GPIO.PWM(RIGHT_MOTOR_ENABLE_ON, 50)
    # r = GPIO.PWM(RIGHT_MOTOR_A, 0.5)
    # p.start(1)
    # r.start(1)

    minimal_publisher = TwistCallback()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()



if __name__ == '__main__':
    main()
