from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
import tf2_ros
from math import sin, cos, atan2, isclose
from transformations import quaternion_from_euler

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

    odom_pub = node.create_publisher(
        Odometry, '/odom', qos_profile=qos_profile_services_default)
    odom_broadcaster = tf2_ros.TransformBroadcaster(node, qos_profile_services_default)

    x = 0.0
    y = 0.0
    th = 0.0
    WHEEL_BASE = 0.177
    while True:
      bytesToRead = ser.inWaiting()
      if bytesToRead > 0:
        
        line = str(ser.readline(), 'ascii')
        parts = line.split(",")

        try:
          dt = int(parts[0]) / 1000.0
          v_left = float(parts[1]) / 1000.0
          v_right = float(parts[2]) / 1000.0
          vx = (v_right + v_left) / 2.0
          vy = 0.0
          vth = (v_right - v_left) / WHEEL_BASE
          
          delta_x = (vx * cos(th) - vy * sin(th)) * dt
          delta_y = (vx * sin(th) + vy * cos(th)) * dt
          delta_th = vth * dt

          x += delta_x
          y += delta_y
          th += delta_th

#          if not isclose(v_left, 0, rel_tol=1e-6):
#            node.get_logger().info('VLeft: {:f} VRight {:f} X: {:f} Y: {:f} TH: {:f}, VTH: {:f}'.format(v_left, v_right, delta_x, delta_y, delta_th, vth))

          odom_quat = quaternion_from_euler(th, 0, 0, axes='sxyz')
          current_time = node.get_clock().now().to_msg()
          
          t = TransformStamped()
          t.header.stamp = current_time
          t.header.frame_id = "odom"
          t.child_frame_id = "base_link"
          t.transform.translation.x = x
          t.transform.translation.y = y
          t.transform.translation.z = 0.0
          t.transform.rotation.x = odom_quat[0]
          t.transform.rotation.y = odom_quat[1]
          t.transform.rotation.z = odom_quat[2]
          t.transform.rotation.w = odom_quat[3]

          odom_broadcaster.sendTransform(t)

          odom = Odometry()
          odom.header.frame_id = "odom"
          odom.header.stamp = current_time

          # set the position
          odom.pose.pose.position.x = x
          odom.pose.pose.position.y = y
          odom.pose.pose.position.z = 0.0
          odom.pose.pose.orientation.x = odom_quat[0]
          odom.pose.pose.orientation.y = odom_quat[1]
          odom.pose.pose.orientation.z = odom_quat[2]
          odom.pose.pose.orientation.w = odom_quat[3]

#          if not isclose(v_left, 0, rel_tol=1e-6):
#            print('orientation: x: {:f}, y: {:f}, z: {:f}, w: {:f}'.format(odom_quat[0], odom_quat[1], odom_quat[2], odom_quat[3]))
    
          # set the velocity
          odom.child_frame_id = "base_link"
          odom.twist.twist.linear.x = vx
          odom.twist.twist.linear.y = vy
          odom.twist.twist.angular.z = vth

          odom_pub.publish(odom)          
        except Exception as ex:
          node.get_logger().error(ex)

      time.sleep(.001)

def main(args=None):
  rclpy.init(args=args)

  with serial.Serial('/dev/ttyARDUINO') as ser:
    twist_subscriber = TwistSubscriber(ser)
    t = threading.Thread(target=serial_reader, args=(twist_subscriber, ser,))
    t.start()
    rclpy.spin(twist_subscriber)
    twist_subscriber.destroy_node()
    rclpy.shutdown()