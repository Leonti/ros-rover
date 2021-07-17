from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
import tf2_ros
from math import sin, cos, atan2, isclose
from hardware_control.transformations_new import quaternion_from_euler
#from bumper_interfaces.msg import Bumper
from std_msgs.msg import String
import queue

import threading
import time

WHEELBASE = 190

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

    self.is_running = True
#    self.last_twist = None
    self.odom_queue = queue.Queue()

  def listener_callback(self, twist):
    linear = twist.linear.x
    angular = twist.angular.z
    if twist.linear.y > 0:
      self.get_logger().error("Can't process 'y' speed - the robot - non-holonomic")
 
#    self.get_logger().warn('linear: "%f", angular "%f"' % (linear, angular))
    linear_mm_sec = linear * 1000

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
    self.command_pub.publish(msg)
#    self.last_twist = (time.time(), right_wheel_speed_mm_sec, left_wheel_speed_mm_sec)
    self.odom_queue.put((time.time(), right_wheel_speed_mm_sec, left_wheel_speed_mm_sec))

def history_to_intervals(history, current_time):
  last_twist_start = None
  intervals = []
  for twist_start in history:
    if last_twist_start is not None:
      (start, right_wheel_speed_mm_sec, left_wheel_speed_mm_sec) = last_twist_start
      (end, _, _) = twist_start
      intervals.append((end - start, right_wheel_speed_mm_sec, left_wheel_speed_mm_sec))    
    last_twist_start = twist_start
  (start, right_wheel_speed_mm_sec, left_wheel_speed_mm_sec) = last_twist_start
  intervals.append((current_time - start, right_wheel_speed_mm_sec, left_wheel_speed_mm_sec))
  return intervals

def odometry_publisher(node):
    node.get_logger().warn("Publishing odometry")

    odom_pub = node.create_publisher(
        Odometry, '/odom', qos_profile=qos_profile_services_default)
    odom_broadcaster = tf2_ros.TransformBroadcaster(node, qos_profile_services_default)

    last_twist = None
    last_msg_received = None
    x = 0.0
    y = 0.0
    th = 0.0
    vx = 0.0
    vy = 0.0
    vth = 0.0
    WHEEL_BASE = WHEELBASE / 1000.0
    INTERVAL_SEC = 0.005
    while node.is_running:
        current_time = time.time()
        if last_msg_received is not None and current_time - last_msg_received >= 2.5:
          last_twist = None

        history = [] if last_twist is None else [last_twist]
        while not node.odom_queue.empty():
            twist = node.odom_queue.get()
            last_msg_received = twist[0]
            history.append(twist)

        intervals = []
        if len(history) > 0:
            intervals = history_to_intervals(history, current_time)
            (_, right_wheel_speed_mm_sec, left_wheel_speed_mm_sec) = history[-1]
            last_twist = (current_time, right_wheel_speed_mm_sec, left_wheel_speed_mm_sec)
            #print("INTERVALS")
            #print(intervals)

        for interval in intervals:
            (dt, right_wheel_speed_mm_sec, left_wheel_speed_mm_sec) = interval

            # fix left/right issue
            v_right = left_wheel_speed_mm_sec / 1000.0
            v_left = right_wheel_speed_mm_sec / 1000.0
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

        odom_quat = quaternion_from_euler(0, 0, th)
        transform_time = node.get_clock().now().to_msg()
        
        t = TransformStamped()
        t.header.stamp = transform_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_quat[0]
        t.transform.rotation.y = odom_quat[1]
        t.transform.rotation.z = odom_quat[2]
        t.transform.rotation.w = odom_quat[3]

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = transform_time

        # set the position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        odom_pub.publish(odom)
        odom_broadcaster.sendTransform(t)

        time.sleep(INTERVAL_SEC)

def main(args=None):
    rclpy.init(args=args)
    twist_subscriber = TwistSubscriber()
    t = threading.Thread(target=odometry_publisher, args=(twist_subscriber,))
    t.start()
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
