import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default

from std_msgs.msg import String

import serial
import threading
import time
from queue import Queue

class CommandSubscriber(Node):

  def __init__(self, ser, q):
    super().__init__('pico_bridge')
    self.subscription = self.create_subscription(
      String,
      '/pico_command',
      self.listener_callback,
      10)
    self.subscription  # prevent unused variable warning

    self.ser = ser
    self.q = q
    self.is_running = True
    self.lock = threading.Lock()

  def listener_callback(self, command):
#    print(command.data)
    self.q.put((command.data + '\n').encode(encoding = 'ascii'))
#    self.lock.acquire()
#    self.ser.write((command.data + '\n').encode(encoding = 'ascii'))
#    self.ser.flush()
#    self.lock.release()

def serial_reader(node, ser):
    node.get_logger().warn("Reading serial")

    publisher = node.create_publisher(
        String, '/pico_output', qos_profile=qos_profile_services_default)

    while node.is_running:
      bytesToRead = ser.inWaiting()
      if bytesToRead > 0:
      
        try:
          line = str(ser.readline(), 'ascii').rstrip()
          msg = String()
          msg.data = line
#          node.get_logger().warn("Read from Pico '{0}'".format(line))
          publisher.publish(msg)
        except Exception as ex:
          node.get_logger().error("Exception: {0}".format(ex))

      time.sleep(.001)

def serial_sender(node, ser, q):
  node.get_logger().warn("Sending messages from a queue")

  while node.is_running:
    message = q.get()
    print(message)
    ser.write(message)
    ser.flush()

def keepalive(node, ser, q):
  node.get_logger().warn("Sending keepalive serial")

  while node.is_running:
#    node.lock.acquire()
#    ser.write(('C3\n').encode(encoding = 'ascii'))
#    ser.flush()
#    node.lock.release()
#    print("PUTTING C3 ON THE QUEUE")
    q.put(('C3\n').encode(encoding = 'ascii'))
#    print("DONE PUTTING C3")
    time.sleep(1.5)

def main():
  rclpy.init()

  q = Queue()
# /dev/ttyUSB0
  with serial.Serial(port = '/dev/serial0', baudrate=115200) as ser:
    command_subscriber = CommandSubscriber(ser, q)
    serial_reader_thread = threading.Thread(target=serial_reader, args=(command_subscriber, ser,))
    serial_reader_thread.start()

    serial_sender_thread = threading.Thread(target=serial_sender, args=(command_subscriber, ser, q))
    serial_sender_thread.start()

    keepalive_thread = threading.Thread(target=keepalive, args=(command_subscriber, ser, q))
    keepalive_thread.start()
    try:
        rclpy.spin(command_subscriber)
    except KeyboardInterrupt:
        command_subscriber.get_logger().warn("Closed with Ctrl-C")
    finally:    
        command_subscriber.is_running = False
        command_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
