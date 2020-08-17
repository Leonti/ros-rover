import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from bumper_interfaces.msg import Bumper
import threading

import RPi.GPIO as GPIO  
GPIO.setmode(GPIO.BCM)

import serial
import threading
import time

class ButtonHandler(threading.Thread):
  def __init__(self, pin, func, bouncetime=200):
    super().__init__(daemon=True)

    self.func = func
    self.pin = pin
    self.bouncetime = float(bouncetime)/1000

    self.lastpinval = GPIO.input(self.pin)
    self.lock = threading.Lock()

  def __call__(self, *args):
    if not self.lock.acquire(blocking=False):
        return

    t = threading.Timer(self.bouncetime, self.read, args=args)
    t.start()

  def read(self, *args):
    pinval = GPIO.input(self.pin)

    if pinval == 0 and self.lastpinval == 1:
      self.func(*args)

    self.lastpinval = pinval
    self.lock.release()


class BumperPublisher(Node):

  def __init__(self):
    super().__init__('bumper_events_publisher')
    self.publisher_ = self.create_publisher(
        Bumper, '/bumper', qos_profile=qos_profile_services_default)

    GPIO.setup(0, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    left_bumper_cb = ButtonHandler(0, self.on_left_bumper, bouncetime=60)
    left_bumper_cb.start()
    GPIO.add_event_detect(0, GPIO.BOTH, callback=left_bumper_cb)

    GPIO.setup(2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    right_bumper_cb = ButtonHandler(2, self.on_right_bumper, bouncetime=60)
    right_bumper_cb.start()
    GPIO.add_event_detect(2, GPIO.BOTH, callback=right_bumper_cb)

  def on_left_bumper(self):
    msg = Bumper()
    msg.center = False
    msg.left = True
    msg.right = False

    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s,%s,%s"' % (msg.left, msg.center, msg.right))

  def on_right_bumper(self):
    msg = Bumper()
    msg.center = False
    msg.left = False
    msg.right = True

    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s,%s,%s"' % (msg.left, msg.center, msg.right))

def main(args=None):
  rclpy.init(args=args)

  bumper_publisher = BumperPublisher()

  rclpy.spin(bumper_publisher)

  bumper_publisher.destroy_node()
  rclpy.shutdown()
    
