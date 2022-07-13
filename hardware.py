import serial
import time
import cv2
import os
import io
from datetime import datetime
import numpy as np

base_path = '/home/pi/Documents/TC_22s/Webcam'

lv_fast = 2000
lv_skip = 20000
lv_slow = 800
lp0 = 0
lp1 = 40
lp_s = 41
lp2 = 105
lp3 = 150

global imcount
imcount = 0

#webcam
resW = 1920
resH = 1080

board = serial.Serial('/dev/ttyUSB0', 115200)
webcam = cv2.VideoCapture(0)




def white_balance(img):
    result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
    return result

def takePicture(dir_name, file_name):
  global imcount
  file_name = imcount
  imcount =+1
  contador = 0
  while True:
    check, frame = webcam.read()
    #cv2.imshow("Current_Shoe", frame)
    key = cv2.waitKey(1)
    if contador==1:                     #Low contador means low light
      file_path = os.path.join(base_path, str(dir_name), str(file_name) + ".png")
      print(file_path)
      img_wb=white_balance(frame)
      cv2.imwrite(filename=file_path, img=img_wb)
      break
    contador=contador+1



def gcode(G):
  print(G)
  board.write(str.encode(G+'\r\n'))

  while True:
    line = board.readline()
    if line == b'ok\n':
      print('DONE')
      break

    print(line)
    #print(linestr[5+linestr.find("echo"):-3])


def initialize():
  time.sleep(2)
  print('Connection to %s established' % board.name)
  gcode('M503')
  time.sleep(1)

  #setup for the printer: Z is table. E is lever. X is camera
  #configuration
  gcode('M92 Z8.888') # Set X-Axis Steps/mm so 1mm equals 1 deg of rotation
  gcode('M203 X180.00 Y100.00 Z800.00 E40000.00') #limit velocity (in mm/s) - G1 is in mm/min
  gcode('M201 X100.00 Y15.00 Z30.00 E10000.00') #limit acceleration

  gcode('G4 P100')
  #turn on lights
  gcode('M140 S100')
  gcode('G4 P100')

  webcam.set(3, resW) #resolution width
  webcam.set(4, resH) #resolution height


def lever(pos, vel):
  gcode('G1 E%i F%i'%(pos,vel))


def collectShoes():
  #push lever to ground fast
  lever(lp1,lv_fast)
  #shoe to middle slower
  lever(lp2,lv_slow)
  #out of image fast
  lever(lp1,lv_fast)


def kickShoes():
  lever(lp2,lv_fast)
  #push out of box slower
  lever(lp3,lv_slow)
  #lever to start position
  lever(lp0,lv_fast)


def scan(dir_name):
  #rotate and take pics
  #von oben
  gcode('G4 P100')
  takePicture(dir_name)

  lever(lp_s, lv_skip)
  lever(lp1, lv_skip)
  gcode('G4 P100')
  #kamera auf 90
  time.sleep(2)

  gcode('G4 P100')

  lever(lp_s, lv_skip)
  lever(lp1, lv_skip)
  gcode('G4 P100')

  takePicture(dir_name)

  gcode('G1 Z90 F1800')
  gcode('G4 P100')

  takePicture(dir_name)

  gcode('G1 Z180 F1800')
  gcode('G4 P100')

  takePicture(dir_name)

  gcode('G1 Z270 F1800')
  gcode('G4 P100')

  takePicture(dir_name)



  lever(lp_s, lv_skip)
  lever(lp1, lv_skip)

  gcode('G1 Z315 F1800')
  gcode('G4 P100')

  time.sleep(1)

  lever(lp_s, lv_skip)
  lever(lp1, lv_skip)

  takePicture(dir_name)


  gcode('G1 Z225 F1800')
  gcode('G4 P100')

  takePicture(dir_name)

  gcode('G1 Z135 F1800')
  gcode('G4 P100')

  takePicture(dir_name)

  gcode('G1 Z45 F1800')
  gcode('G4 P100')

  takePicture(dir_name)

  lever(lp_s, lv_skip)
  lever(lp1, lv_skip)


  gcode('G1 Z0 F1800')
  gcode('G4 P100')




num_shoes = 1

for i in range(num_shoes):
  now = datetime.now()
  dir_name = now.strftime("%y%m%d-%H%M%S")
  dir_path = os.path.join(base_path, dir_name)
  os.mkdir(dir_path)
  initialize()
  collectShoes()
  scan(dir_name)
  kickShoes()


#lever to shoe fast


input('Enter to close script')


webcam.release()
board.close()


