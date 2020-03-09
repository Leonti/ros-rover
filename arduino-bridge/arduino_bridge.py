from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default

import serial
import threading
import time

class TwistSubscriber(Node):

  def __init__(self, ser):
    super().__init__('arduino_bridge')
    self.subscription = self.create_subscription(
      Twist,
      '/cmd_vel',
      self.listener_callback,
      10)
    self.subscription  # prevent unused variable warning
    self.ser = ser

  def listener_callback(self, twist):
    linear = twist.linear.x
    angular = twist.angular.z
    if twist.linear.y > 0:
      self.get_logger().error("Can't process 'y' speen - the robot - non-holonomic")
    self.get_logger().warn('linear: "%f", angular "%f"' % (linear, angular))
    linear_mm_sec = linear * 1000
    self.ser.write('S{:f} {:f} '.format(linear_mm_sec, angular).encode(encoding = 'ascii'))
    self.ser.flush()

def serial_reader(node, ser):
    node.get_logger().warn("Reading serial")
#    while True:
#      bytesToRead = ser.inWaiting()
#      if bytesToRead > 0:
#        line = ser.readline()
#        node.get_logger().warn(str(line, 'ascii'))
#      time.sleep(.001)

def main(args=None):
  rclpy.init(args=args)

  with serial.Serial('/dev/ttyARDUINO') as ser:
    twist_subscriber = TwistSubscriber(ser)
    t = threading.Thread(target=serial_reader, args=(twist_subscriber, ser,))
    t.start()
    rclpy.spin(twist_subscriber)
    twist_subscriber.destroy_node()
    rclpy.shutdown()