import serial
import io
import threading
import time
import statistics
import pprint

pp = pprint.PrettyPrinter(indent=4)

is_recording = False
linear_speed = 0
readings = []

def serial_reader(ser):
    global readings
    global is_recording
    while True:
      try:
        bytesToRead = ser.inWaiting()
      except Exception as ex:
        print(ex)
        raise ex
      if bytesToRead > 0:
        try:
#          line = sio.readline()
          line = str(ser.readline(), 'ascii')
#          print(line)
          parts = line.split(",")

          dt = int(parts[0]) / 1000.0
          v_left = float(parts[1]) / 1000.0
          v_right = float(parts[2]) / 1000.0

          ticks_left = float(parts[5])
          ticks_right = float(parts[6])

          if is_recording and float(parts[1]) > 0:
            readings.append({ 
              'left': float(parts[1]), 
              'right': float(parts[2]),
              'ticks_left': ticks_left, 
              'ticks_right': ticks_right,
              })

        except Exception as ex:
          print(ex)

def serial_writer(ser):
  global linear_speed
  while True:
    linear_mm_sec = linear_speed * 1000
    #if linear_speed > 0:
      #print('S{:f} {:f} '.format(linear_mm_sec, 0))
    ser.write('R{:f} {:f} '.format(0.61, 0.62).encode(encoding = 'ascii'))
    ser.write('S{:f} {:f} '.format(linear_mm_sec, 0).encode(encoding = 'ascii'))
    ser.flush()
    time.sleep(0.5)

def controller(ser):
  global readings
  global is_recording
  global linear_speed
  for kp in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]:
    for kd in [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07]:
      for i in [1, 2]:
        ki = 0.0
        ser.write('T{:f} {:f} {:f} '.format(kp, kd, ki).encode(encoding = 'ascii'))
        readings = []
        linear_speed = 0.15
        time.sleep(2)
        is_recording = True
        time.sleep(15)
        linear_speed = 0
        is_recording = False
        left_readings = [d['ticks_left'] for d in readings]
        right_readings = [d['ticks_right'] for d in readings]
      #  pp.pprint(left_readings)
        left_std_dev = statistics.stdev(left_readings)
        right_std_dev = statistics.stdev(right_readings)
        left_mean = statistics.mean(left_readings)
        right_mean = statistics.mean(right_readings)  
        print('KP: {:f}, KD: {:f}, STDDEV left: {:f}, right: {:f}. MEAN left: {:f}, right: {:f}'.format(kp, kd, left_std_dev, right_std_dev, left_mean, right_mean))

def main(args=None):
  print("Starting...") # 256000
  with serial.Serial(port = '/dev/ttyUSB0', baudrate=115200) as ser:
    time.sleep(2)
#    sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser, 1), 'ascii')
    t = threading.Thread(target=serial_reader, args=(ser,))
    t1 = threading.Thread(target=serial_writer, args=(ser,))
    t2 = threading.Thread(target=controller, args=(ser,))
    t.start()
    t1.start()
    t2.start()
    t.join()
    t1.join()
    t2.join()
#    serial_reader(ser)

main()   