import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import RPi.GPIO as GPIO
import numpy as np

import time
from math import cos, sin
import math

# GPIO pins for the encoders. Note that they are named after the board
LEFT_ENCODER_GPIO = 37
RIGHT_ENCODER_GPIO = 35

# GPIO pins for the right motor. Note that they are named after the board
RIGHT_MOTOR_ENABLE_ON=15
RIGHT_MOTOR_A = 13
RIGHT_MOTOR_B = 11

# GPIO pins for the left motor. Note that they are named after the board
LEFT_MOTOR_ENABLE_ON=22
LEFT_MOTOR_A = 16
LEFT_MOTOR_B = 18


TICKS_PER_REVOLUTION = 1992 * 2 # Note: multiplying by 2 right now because I fire an interrupt on both rising and falling edge
DISTANCE_BETWEEN_WHEELS_M = 0.26
WHEEL_CIRCUMFERENCE_MM = 217

POSE_TOPIC = '/odom'

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class ViamRobotController(Node):
    def __init__(self):
        super().__init__('viam_robot_controller')
        self.get_logger().info("Viam robot controller initialising")

        # Set up the publisher for the odometry
        # We update the odometry at a fixed rate, and broadcast the transform as well. 
        self.odometry_publisher = self.create_publisher(Odometry, '/my_odom' , 10)
        self.wheel_tick_publisher = self.create_publisher(String, '/wheel_ticks', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the velocity topic. We try to keep the last velocity message in memory and keep up with that. 
        self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)
        self.last_velocity = None

        # Set up the timer for both the motor control and odometry update
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.motor_control_callback)
        self.odometry_update_timer = self.create_timer(timer_period, self.odometry_update)

        # Set up the GPIO nodes for controlling the motors
        # We control them by adjusting the PWM frequency and keep track of the pwm we set... 
        self.last_pwm_left = 0
        self.last_pwm_right = 0

        # Variables for the encoder values. We keep track of the encoder values and the direction of the wheels
        self.left_encoder = 0
        self.right_encoder = 0
        self.left_wheel_forward = True
        self.right_wheel_forward = True
        self.last_encoder_left = 0
        self.last_encoder_right = 0

        # Get the current time and keep track of it
        self.last_time = self.get_clock().now()

        # Variables for odometry calculation - we assume start position is 0,0 and orientation is 0
        self.position_x = 0 
        self.position_y = 0
        self.orientation = 0

    def left_encoder_callback(self, channel):
        """Callback function for the left encoder. """
        
        if GPIO.input(LEFT_ENCODER_GPIO):
            if self.left_wheel_forward:
                self.left_encoder += 1
            else:
                self.left_encoder -= 1

    def right_encoder_callback(self, channel): 
        """Callback function for the right encoder. """
        if GPIO.input(RIGHT_ENCODER_GPIO): 
            if self.right_wheel_forward:
                self.right_encoder += 1
            else:
                self.right_encoder -= 1


    def velocity_callback(self, msg):
        """Callback function for the velocity message. """
        self.last_velocity = msg

    def publish_odometry_message(self, timestamp, velocity_xy, velocity_theta): 
        odometry_message = Odometry()
        odometry_message.header.stamp = timestamp.to_msg()
        odometry_message.header.frame_id = 'roland_odom'
        odometry_message.child_frame_id = 'base_link'
        
        odometry_message.pose.pose.position.x = self.position_x 
        odometry_message.pose.pose.position.y = self.position_y
        odometry_message.pose.pose.position.z = 0.0
        odometry_message.pose.pose.orientation.x = 0.0
        odometry_message.pose.pose.orientation.y = 0.0
        odometry_message.pose.pose.orientation.z = sin(self.orientation / 2)

        odometry_message.twist.twist.linear.x = velocity_xy
        odometry_message.twist.twist.linear.y = 0.0
        odometry_message.twist.twist.linear.z = 0.0
        odometry_message.twist.twist.angular.x = 0.0
        odometry_message.twist.twist.angular.y = 0.0
        odometry_message.twist.twist.angular.z = velocity_theta

        self.odometry_publisher.publish(odometry_message)

    def publish_transform(self, timestamp):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.position_x
        t.transform.translation.y = self.position_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sin(self.orientation / 2)

        q = quaternion_from_euler(0, 0, self.orientation)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def odometry_update(self):
        """Update the odometry based on the encoder values. """        
        timestamp = self.get_clock().now()

        # Get the time since last update and convert it to seconds
        delta_time = (timestamp - self.last_time)
        delta_time = delta_time.nanoseconds / 1e9

        if delta_time == 0:
            return

        left_ticks = self.left_encoder - self.last_encoder_left
        right_ticks = self.right_encoder - self.last_encoder_right

        msg = String()
        msg.data = f"Left: {left_ticks}, Right: {right_ticks}"
        self.wheel_tick_publisher.publish(msg)
        
        left_distance = left_ticks / TICKS_PER_REVOLUTION
        right_distance = right_ticks / TICKS_PER_REVOLUTION

        left_speed = left_distance / delta_time
        right_speed = right_distance / delta_time

        delta_xy = (left_distance + right_distance) / 2
        delta_theta = (left_distance - right_distance) / DISTANCE_BETWEEN_WHEELS_M
        velocity_xy = delta_xy / delta_time
        velocity_theta = delta_theta / delta_time

        self.orientation += delta_theta
        self.position_x += delta_xy * cos(self.orientation)
        self.position_y += delta_xy * sin(self.orientation)

        # self.get_logger().info(f"Position x: {self.position_x:.2f}, y: {self.position_y:.2f}, theta: {self.orientation:.2f}")

        # Publish odometry message and the transform
        self.publish_odometry_message(timestamp, velocity_xy, velocity_theta)        
        self.publish_transform(timestamp)

        # Update variables for next call
        self.last_encoder_left = self.left_encoder
        self.last_encoder_right = self.right_encoder
        self.last_time = timestamp

    def motor_control_callback(self):
        global high_on
        
        if self.last_velocity != None:
            left_motor_speed = 0
            right_motor_speed = 0
            forward_speed = self.last_velocity.linear.x
            angular_speed = self.last_velocity.angular.z
            wheel_circumference_mm = 217
            width_mm = 260
            MAX_PWM = 100

            # Calculate left motor speed and right motor speed
            left_motor_speed = forward_speed + angular_speed * width_mm / 2
            right_motor_speed = forward_speed - angular_speed * width_mm / 2

            # self.get_logger().info(f"Left motor speed: {left_motor_speed:.2f}, right motor speed: {right_motor_speed:.2f}")

            if left_motor_speed > 0:
                GPIO.output(LEFT_MOTOR_A, GPIO.HIGH)
                GPIO.output(LEFT_MOTOR_B, GPIO.LOW)
                self.left_wheel_forward = True
            elif left_motor_speed < 0:
                GPIO.output(LEFT_MOTOR_A, GPIO.LOW)
                GPIO.output(LEFT_MOTOR_B, GPIO.HIGH)
                self.left_wheel_forward = False
            else:
                GPIO.output(LEFT_MOTOR_A, GPIO.LOW)
                GPIO.output(LEFT_MOTOR_B, GPIO.LOW)
            
            left_motor_pwm = abs(left_motor_speed * 50)
            if left_motor_pwm != 0:
                left_motor_pwm = max(1, min(left_motor_pwm, MAX_PWM))
                if left_motor_pwm != self.last_pwm_left:
                    self.last_pwm_left = left_motor_pwm
                    self.pwm_left.ChangeFrequency(left_motor_pwm)

            
            if right_motor_speed > 0:
                GPIO.output(RIGHT_MOTOR_A, GPIO.LOW)
                GPIO.output(RIGHT_MOTOR_B, GPIO.HIGH)
                self.right_wheel_forward = True
            elif right_motor_speed < 0:
                GPIO.output(RIGHT_MOTOR_A, GPIO.HIGH)
                GPIO.output(RIGHT_MOTOR_B, GPIO.LOW)
                self.right_wheel_forward = False
            else:
                GPIO.output(RIGHT_MOTOR_A, GPIO.LOW)
                GPIO.output(RIGHT_MOTOR_B, GPIO.LOW)
            
            right_motor_pwm = abs(right_motor_speed * 50)
            if right_motor_pwm != 0:
                
                right_motor_pwm = max(1, min(right_motor_pwm, MAX_PWM))
                if right_motor_pwm != self.last_pwm_right:
                    self.last_pwm_right = right_motor_pwm
                    self.pwm_right.ChangeFrequency(right_motor_pwm)
     



def main(args=None):
    # Set up ROS
    rclpy.init(args=args)

    # Set up the GPIO nodes for odometry and motors. 
    # We use the board setting for the GPIO pins so it's easier to reference on the raspberry pi
    GPIO.setmode(GPIO.BOARD)

    viam_robot_controller = ViamRobotController()

    # Set up the odometry encoders and the callback functions
    GPIO.setup(LEFT_ENCODER_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LEFT_ENCODER_GPIO, GPIO.BOTH, callback=viam_robot_controller.left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER_GPIO, GPIO.BOTH, callback=viam_robot_controller.right_encoder_callback)
    
    # Set up the motor control GPIO
    GPIO.setup(RIGHT_MOTOR_ENABLE_ON, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_A, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_B, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_ENABLE_ON, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_A, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_B, GPIO.OUT)    

    viam_robot_controller.pwm_left = GPIO.PWM(LEFT_MOTOR_ENABLE_ON, 10)
    viam_robot_controller.pwm_right = GPIO.PWM(RIGHT_MOTOR_ENABLE_ON, 10)
    viam_robot_controller.pwm_left.start(50)
    viam_robot_controller.pwm_right.start(50)

    # Set up the robot controller node and spin it
    rclpy.spin(viam_robot_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
