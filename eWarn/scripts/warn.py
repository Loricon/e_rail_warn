#-*- coding: UTF-8 -*- 

import os
import time
from datetime import datetime
from playsound import playsound
import rospy
from std_msgs.msg import String

def warn():
  rospy.init_node('Warn',anonymous=False)
  rospy.Subscriber('warn',String,callback,queue_size=1,buff_size=52428800)
  print("waiting warn class ...")
  rospy.spin()
    

def callback(data):
  
  warnClass, angleClass = eval(data.data)
  print("distance warning mode: "+str(warnClass)+",angle warn mode: "+str(angleClass))

  if warnClass == 0:
    pass
  elif warnClass == 1:
    while True:
      playsound(os.environ['HOME']+'/e_rail_warn/eWarn/resource/di1.wav')
#      time.sleep(1.5)
      break
  elif warnClass == 2:
    while True:
      playsound(os.environ['HOME']+'/e_rail_warn/eWarn/resource/di2.wav')
#      time.sleep(0.8)
      break
  elif warnClass == 3:
    while True:
      playsound(os.environ['HOME']+'/e_rail_warn/eWarn/resource/di3.wav')
      break

  if angleClass == 0:
    pass
  elif angleClass == 1:
    while True:
      playsound(os.environ['HOME']+'/e_rail_warn/eWarn/resource/di1.wav')
#      time.sleep(1.5)
      break
  elif angleClass == 2:
    while True:
      playsound(os.environ['HOME']+'/e_rail_warn/eWarn/resource/di2.wav')
#      time.sleep(0.8)
      break
  elif angleClass == 3:
    while True:
      playsound(os.environ['HOME']+'/e_rail_warn/eWarn/resource/di3.wav')
      break

if __name__ == '__main__':
  warn()
