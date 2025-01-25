import rclpy
from rclpy.node import Node
import time
import RPi.GPIO as GPIO

from std_msgs.msg import String

LEFT_ENCODER_GPIO = 19
RIGHT_ENCODER_GPIO = 26

RIGHT_MOTOR_ENABLE_ON=22
RIGHT_MOTOR_A = 16
RIGHT_MOTOR_B = 18

left_encoder = 0
right_encoder = 0

def left_encoder_callback(channel):
    global left_encoder
    if GPIO.input(LEFT_ENCODER_GPIO):
        left_encoder += 1

def right_encoder_callback(channel): 
    global right_encoder
    if GPIO.input(RIGHT_ENCODER_GPIO): 
        right_encoder += 1


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'random_publish_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        print("minimal published init")
        self.i = 0

    def timer_callback(self):
        global high_on
        print("hello world", left_encoder, right_encoder)
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    # Set up ROS
    rclpy.init(args=args)

    # Set up the GPIO nodes for odometry
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(LEFT_ENCODER_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LEFT_ENCODER_GPIO, GPIO.BOTH, callback=left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER_GPIO, GPIO.BOTH, callback=right_encoder_callback)
    
    
    RIGHT_MOTOR_ENABLE_ON=22
    RIGHT_MOTOR_A = 16
    RIGHT_MOTOR_B = 18
    GPIO.setup(RIGHT_MOTOR_ENABLE_ON, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_A, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_B, GPIO.OUT)
    p = GPIO.PWM(RIGHT_MOTOR_ENABLE_ON, 50)
    r = GPIO.PWM(RIGHT_MOTOR_A, 0.5)
    p.start(1)
    r.start(1)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()



if __name__ == '__main__':
    main()
