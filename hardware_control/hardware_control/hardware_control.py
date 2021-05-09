from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
import tf2_ros
from math import sin, cos, atan2, isclose
from transformations_new import quaternion_from_euler
#from bumper_interfaces.msg import Bumper
from std_msgs.msg import String

import threading
import time

class TwistSubscriber(Node):

  def __init__(self):
    super().__init__('hardware_control')
    self.twist_subscription = self.create_subscription(
      Twist,
      '/cmd_vel',
      self.listener_callback,
      10)

    self.command_pub = self.create_publisher(
        String, '/pico_command', qos_profile=qos_profile_services_default)

#    self.handling_bumper = False

  def listener_callback(self, twist):
    linear = twist.linear.x
    angular = twist.angular.z
    if twist.linear.y > 0:
      self.get_logger().error("Can't process 'y' speed - the robot - non-holonomic")
 
#    self.get_logger().warn('linear: "%f", angular "%f"' % (linear, angular))
    linear_mm_sec = linear * 1000

    WHEELBASE = 175
    right_wheel_speed_mm_sec = linear_mm_sec - angular * (WHEELBASE / 2)
    left_wheel_speed_mm_sec = linear_mm_sec + angular * (WHEELBASE / 2)

    WHEEL_CIRCUMFERENCE_MM = 210.49 #// 33.5 * PI * 2
    # 6400 steps for a full revolution
    steps_per_mm = 6400 / WHEEL_CIRCUMFERENCE_MM

    right_speed_steps = -steps_per_mm * right_wheel_speed_mm_sec
    left_speed_steps = steps_per_mm * left_wheel_speed_mm_sec
    command = "C1:1,%d,%d" % (right_speed_steps, left_speed_steps)

    msg = String()
    msg.data = command
#    self.get_logger().warn(f"command '{command}'")
    self.command_pub.publish(msg)
#      linear_mm_sec = linear * 1000
#      self.ser.write('S{:f} {:f} '.format(linear_mm_sec, angular).encode(encoding = 'ascii'))
#      self.ser.flush()

def main(args=None):
    rclpy.init(args=args)
    twist_subscriber = TwistSubscriber()
#    t = threading.Thread(target=serial_reader, args=(twist_subscriber, ser,))
#    t.start()
    try:
        rclpy.spin(twist_subscriber)
    except:
        twist_subscriber.get_logger().warn("Exception: during node close")
    finally:    
        twist_subscriber.is_running = False
        twist_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
