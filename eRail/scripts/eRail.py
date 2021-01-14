#!/bin/python
#-*- coding: UTF-8 -*- 

# 导入除法
from __future__ import division
# 系统库
import os
import copy
import time
from datetime import datetime
from playsound import playsound
# ROS
import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
# 画图
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Arrow
# 消息格式
from geometry_msgs.msg import Pose2D
from novatel_gps_msgs.msg import Inspvax
from std_msgs.msg import String
# 自定义模块
from dep.Vector2d import Vector2d
from dep import GIS
from dep.Excavator import Excavator
from dep.Excavator import Pose3D
from dep.Excavator import Position2D
from dep.utm import wgs84_utm_conversion_2d as w2u

class Rail:
  '''
  定义围栏
  '''
  points_x = list()
  points_y = list()

  def __init__(self,x,y):
    self.points_x = x
    self.points_y = y
  def set(self,x,y):
    self.points_x = x
    self.points_y = y

class ClosedRail(Rail):
  '''
  封闭式围栏
  '''
  if_closed = True
  forbidden = "inside"
  def setForbidden(self,side):
    self.forbidden = side

class OpenedRail(Rail):
  '''
  开放式围栏
  '''
  if_closed = False

class ERail:
  '''
  电子围栏数据结构
  '''
  origin_x = None
  origin_y = None
  debug = False
  test = False
  rails = list()
  check_range = 0
  distance_limit = 0
  log = True
  utm = True
  TF = True

  def __init__(self,c,d,test=False,debug=False,log=True,utm=True,tf=True):
    self.check_range = c
    self.distance_limit = d
    self.debug = debug
    self.test = test
    self.log = log
    self.utm = utm
    self.TF = tf


  def setRails(self,rails):
    self.rails = rails

  def addRail(self,rail):
    self.rails.append(rail)

  def readRails(self):
    rails = list();
    n = input("请输入电子围栏个数:")
    
    # 开始记录
    if self.log:
      dt=datetime.now()
      time = str(dt.strftime( '[%Y-%m-%d-%H:%M:%S] '))
      with open(os.environ['HOME']+'/e_rial_warn/eRail/data/log.log','a') as f:
        f.write(30*'='+'start time: '+time+30*'='+'\n')

    for i in range(n):
      x,y = self.readRailPoints()
      # 创建围栏
      if x is not None and y is not None:
        ifset = raw_input(
            "是否设置围栏属性？默认属性为封闭且内部为不可行驶区域(y/n)")
        if ifset=='y':
          print("请选择\n")
          c = raw_input("\ta. 封闭、内部为不可行驶区域\n\tb. 封闭、外部为不可行驶区域\n\tc. 开放\n")
          if c=='a':
            rail = ClosedRail(x+[x[0]],y+[y[0]])
            rail.setForbidden("inside")
          elif c=='b':
            rail = ClosedRail(x+[x[0]],y+[y[0]])
            rail.setForbidden("outside")
          elif c=='c':
            rail = OpenedRail(x,y)
          self.rails.append(rail)
        elif ifset=='n':
          rail = ClosedRail(x+[x[0]],y+[y[0]])
          self.rails.append(rail)

        # 保存电子围栏数据
        if self.log:
          with open(os.environ['HOME']+'/e_rail_warn/eRail/data/log.log','a') as f:
            f.write('x: ')
            for a in x:
              f.write(str(a)+', ')
            f.write('\ny: ')
            for a in y:
              f.write(str(a)+', ')
            f.write('\n')
      else:
        print("当前电子围栏节点数目过少，将被忽略")
        pass

  def readRailPoints(self):
    """
    输入参数，即电子围栏节点，并转化为局部坐标，以米为单位
    将第一个节点经纬度坐标设置为局部坐标原点。
    """
    if self.debug:
      print("电子围栏使用样例")
      x = [-3,-3,8,8,28,28,24,24,15,15,12,12]
      y = [-3,20,20,28,28,3,3,-3,-3,3,3,-3]
      self.origin_y = 31.4009312396
      self.origin_x = 121.059761073
      return x,y
    else:
      print("手动输入电子围栏节点(经度,纬度)")
      x = list()
      y = list()
      print("请输入电子围栏节点,按回车结束输入:\n")
      i = 0;
      while True:
        i += 1
        lon,lat= self.read_one(i)
        if lon is None:
          break
        else:
          x.append(lon)
          y.append(lat)
      if len(x) > 0 or len(y) > 0:
        self.origin_x,self.origin_y = x[0],y[0]
        return x,y
      else:
        return None,None

  def read_one3(self,i):
    '''
    读取一次
    '''
    p_str = input("点 "+str(i)+": ")
    l = len(p_str)
    # 直接回车表示终止输入
    if l==0:
      stop = input("当前点个数为"+str(i-1)+"，是否结束输入?(y/n)")
      if stop == "n":
        return read_one3(i)
      elif stop == "y":
        return None,None
      else:
        print("y")
        return None,None
    # 输入长度小于3则格式一定错误
    elif l < 3:
      print("输入数据格式错误，请重新输入当前点")
      return read_one3(i)
    # 有逗号才能组成元组
    else:
      k = p_str.find(",")
      if(k<1 or k==l):
        print("输入数据格式错误，请重新输入当前点")
        return read_one3(i)

      if self.utm:
        return w2u(eval(p_str))
      else:
        return eval(p_str)

  def read_one(self,i):
    try:
      lon,lat = input("点 "+str(i)+": ")
    #  b = str(lon)
    #  lon =  eval(b[b.find('.'):])/60 + eval(b[:b.find('.')])
    #  b = str(lat)
    #  lat =  eval(b[b.find('.'):])/60 + eval(b[:b.find('.')])
      if self.utm:
        return w2u(lon,lat)
      else:
        return lon,lat
    except:
      stop = raw_input("当前点个数为"+str(i-1)+"，是否结束输入?(y/n)")
      # 直接回车表示终止输入
      if stop == "y":
        return None,None
      elif stop == "n":
        return self.read_one(i)
      else:
        print("y")
        return None,None

  def drawExcavator(self,excavator,inputs,mode="plot"):
    x = excavator.position.x
    y = excavator.position.y
    yaw = excavator.pose.yaw
    pitch = excavator.pose.pitch
    roll = excavator.pose.roll
    xs_u = excavator.up_outlines_x
    ys_u = excavator.up_outlines_y
    xs_d = excavator.do_outlines_x
    ys_d = excavator.do_outlines_y
    ax = inputs[0]
    ax1 = inputs[1]
    ax2 = inputs[2]
    ax3 = inputs[3]
    ax4 = inputs[4]
    """
    画出挖掘机所在位置
    """
    if mode=="plot":
      if x==0 and y==0:
        p1, = ax.plot([],[],'r.')
        p2, = ax.plot([],[],'k.-')
        p3, = ax.plot([],[],'r-')
        p4, = ax.plot([],[],'b.-')
        text = ax1.text(0.23,0.6, '', fontsize=11)
      else:
        p1, = ax.plot(x,y,'g.')
        p2, = ax.plot(xs_d,ys_d,'k.-')
        alpha = np.linspace(0,2*np.pi,100)
        xr = [self.check_range*np.cos(al)+x for al in alpha]
        yr = [self.check_range*np.sin(al)+y for al in alpha]
        p3, = ax.plot(xr,yr,'r-')
        p4, = ax.plot(xs_u,ys_u,'b.-')
        text = ax1.text(0.23,0.6, '', fontsize=11)

    elif mode=="change":
      if x==0 and y==0:
        return
      p1 = inputs[5]
      p2 = inputs[6]
      p3 = inputs[7]
      p4 = inputs[8]
      text = inputs[9]
      plt_yaw = inputs[10]
      plt_pitch = inputs[11]
      plt_roll = inputs[12]
      
      p1.set_data(x,y)
      p2.set_data(xs_d,ys_d)
      p4.set_data(xs_u,ys_u)

      alpha = np.linspace(0,2*np.pi,100)
      xr = [self.check_range*np.cos(al) for al in alpha]
      yr = [self.check_range*np.sin(al) for al in alpha]

      # 设置实时显示信息
      if self.debug:
        if self.utm:
          text.set_text("x=%.3f, y=%.3f, yaw=%.2f/deg"%(x,y, np.rad2deg(yaw)))
        else:
          text.set_text("x=%.6f, y=%.6f, yaw=%.2f/deg"%(x,y, np.rad2deg(yaw)))
        p3.set_data(xr,yr)
      
      else:
        if self.utm:
          xa = [r+x for r in xr]
          ya = [r+y for r in yr]
          p3.set_data(xa,ya)
          text.set_text("x=%.3f, y=%.3f, yaw=%.2f/deg"%(x,y,yaw))
          self.arrowPlot(plt_yaw,yaw,'yaw','change')
          self.arrowPlot(plt_pitch,pitch,'pitch','change')
          self.arrowPlot(plt_roll,roll,'roll','change')
        else:
          if self.origin_x is not None:
            xa = [GIS.xToLon(i,x,y) for i in xr]
            ya = [GIS.yToLat(j,x,y) for j in yr]
            p3.set_data(xa,ya)
          text.set_text("x=%.6f, y=%.6f, yaw=%.2f/deg"%(x,y,yaw))


    else:
      print("不能识别的模式")
      return
    
    # 自动调整图像大小
    xmin,xmax = ax.get_xlim()
    ymin,ymax = ax.get_ylim()
    if self.utm:
      if len(self.rails) == 0:
        if abs(x-xmin)<self.check_range+5 or x<xmin or abs(x-xmax)<self.check_range+5 or x>xmax or abs(y-ymin)<self.check_range+5 or y<ymin or abs(y-ymax)<self.check_range+5 or y>ymax:
          ax.set(xlim=(x-50,x+50),ylim=(y-50,y+50))

      else:
        if abs(x-xmin)<self.check_range+5 or x<xmin:
          xmin -= 50
          ax.set(xlim=(xmin,xmax))
        if abs(x-xmax)<self.check_range+5 or x>xmax:
          xmax += 50
          ax.set(xlim=(xmin,xmax))
        if abs(y-ymin)<self.check_range+5 or y<ymin:
          ymin -= 50
          ax.set(ylim=(ymin,ymax))
        if abs(y-ymax)<self.check_range+5 or y>ymax:
          ymax += 50
          ax.set(ylim=(ymin,ymax))
    else:
      # 南
      A,B = [xmin,ymin],[xmax,ymin]
      d = GIS.disBetweenPointAndSegment([x,y],A,B,"degree")
      if d<self.check_range+5 or y<ymin:
        ymin -= 0.001
        ax.set(ylim=(ymin,ymax))
      # 北
      A,B = [xmin,ymax],[xmax,ymax]
      d = GIS.disBetweenPointAndSegment([x,y],A,B,"degree")
      if d<self.check_range+5 or y>ymax:
        ymax += 0.001
        ax.set(ylim=(ymin,ymax))
      # 西
      A,B = [xmin,ymin],[xmin,ymax]
      d = GIS.disBetweenPointAndSegment([x,y],A,B,"degree")
      if d<self.check_range+5 or x<xmin:
        xmin -= 0.001
        ax.set(xlim=(xmin,xmax))
      # 东
      A,B = [xmax,ymin],[xmax,ymax]
      d = GIS.disBetweenPointAndSegment([x,y],A,B,"degree")
      if d<self.check_range+5 or x>xmax:
        xmax += 0.001
        ax.set(xlim=(xmin,xmax))

    return p1,p2,p3,p4,text
  
  def drawRail(self,ax):
    """
    画出电子围栏，目前没有考虑是否按照顺序输入。
    """
    # 使用UTM坐标系时，将所有坐标都减掉原点坐标，即转化为局部米坐标，
    # 如果有电子围栏，坐标原点为电子围栏第一个点的位置，
    # 如果没有电子围栏，坐标原点为挖掘机的初始位置，此时在画电子围栏时还未收到
    # 挖掘机初始位置，因此无法转化
    x_max = 0
    x_min = 20e10
    y_max = 0
    y_min = 20e10
    for rail in self.rails:
      x = rail.points_x
      y = rail.points_y
      if x is not None and y is not None:
        if len(x) != len(y):
          print("ERROR(drawRail):\tx和y数据不等长!")
          return
        elif len(x) == 0 or len(y) == 0:
          print("ERROR(drawRail):\t输入数据为空!")
          return
        if rail.if_closed:
          ax.plot(x,y,'k-')
          if rail.forbidden == "inside":
            ax.fill(x,y,"gray")
        else:
          ax.plot(x,y,'k-')
        
        x_max = max(max(x), x_max)
        x_min = min(min(x), x_min)
        y_max = max(max(y), y_max)
        y_min = min(min(y), y_min)
      else:
        if self.debug:
          a = input("电子围栏显示为空! 继续?")
        else:
          pass
    
    if len(self.rails) != 0:
      xmax = x_max + (x_max-x_min)/5
      xmin = x_min - (x_max-x_min)/5
      ymax = y_max + (y_max-y_min)/5
      ymin = y_min - (y_max-y_min)/5
      ax.set(xlim=(xmin,xmax),ylim=(ymin,ymax))
  
  def makeFigure(self):
    '''
    创建图像
    '''
    fig = plt.figure()
    ax = plt.axes([0.05,0.25,0.7,0.72])
    ax.axis("equal")
    
    ax1 = plt.axes([0.05,0.05,0.9,0.15])
    ax1.set(xlim =(0, 1), ylim =(0, 1))
    ax1.text(0.02,0.6,'real time INFO:',fontsize=11)
    plt.xticks([])
    plt.yticks([])
    
    t = np.linspace(0,2*np.pi,30)
    x = np.cos(t)
    y = np.sin(t)

    ax2 = plt.axes([0.77,0.25,0.18,0.24])
    plt.xticks([])
    plt.yticks([])
    ax2.set(xlim=(-1,1),ylim=(-1,1))
    ax2.plot(x,y,'r-')
    ax2.plot([x[0],x[-1]],[y[0],y[-1]],'r-')
    ax2.text(-0.15,-0.95,'-90', fontsize=6, color="k")
    ax2.text(-1,-0.05,'180', fontsize=6, color="k")
    ax2.text(-0.1,0.8,'90', fontsize=6, color="k")
    ax2.text(0.9,-0.05,'0', fontsize=6, color="k")
    ax2.text(-0.15,-0.65,'roll',fontsize=9,color='r')
    roll = self.arrowPlot(ax2,0,'roll','plot')
    
    ax3 = plt.axes([0.77,0.49,0.18,0.24])
    plt.xticks([])
    plt.yticks([])
    ax3.set(xlim=(-1,1),ylim=(-1,1))
    ax3.plot(x,y,'g-')
    ax3.plot([x[0],x[-1]],[y[0],y[-1]],'g-')
    ax3.text(-0.15,-0.95,'-90', fontsize=6, color="k")
    ax3.text(-1,-0.05,'180', fontsize=6, color="k")
    ax3.text(-0.1,0.8,'90', fontsize=6, color="k")
    ax3.text(0.9,-0.05,'0', fontsize=6, color="k")
    ax3.text(-0.3,-0.65,'pitch',fontsize=9,color='g')
    pitch = self.arrowPlot(ax3,0,'pitch','plot')
    
    ax4 = plt.axes([0.77,0.73,0.18,0.24])
    plt.xticks([])
    plt.yticks([])
    ax4.set(xlim=(-1,1),ylim=(-1,1))
    ax4.plot(x,y,'b-')
    ax4.plot([x[0],x[-1]],[y[0],y[-1]],'b-')
    ax4.text(-1,-0.05,'90', fontsize=6, color="k")
    ax4.text(-0.175,-0.9,'180', fontsize=6, color="k")
    ax4.text(-0.06,0.8,'0', fontsize=6, color="k")
    ax4.text(0.75,-0.05,'270', fontsize=6, color="k")
    ax4.text(-0.25,-0.65,'yaw',fontsize=9,color='b')
    yaw = self.arrowPlot(ax4,0,'yaw','plot')

    return fig, ax, ax1, ax2, ax3, ax4, yaw, pitch, roll
  
  def arrowPlot(self,ax,theta,which,mode):
    x0 = 0
    y0 = 0
    x1 = x0+0.9
    y1 = y0
    if which == 'yaw':
      c = 'b'
      theta = 90 + theta
    elif which == 'pitch':
      c = 'g'
    elif which == 'roll':
      c = 'r'
    
    theta = np.deg2rad(theta)
    T = np.array([
        [np.cos(theta),-np.sin(theta)],
        [np.sin(theta),np.cos(theta)]])
    x2,y2 = np.matmul(T,[x1,y1])
    if mode == 'plot':
      arrow, = ax.plot([x0,x2],[y0,y2],c+'-')
      return arrow
    elif mode == 'change':
      ax.set_data([x0,x2],[y0,y2])

  def endplot(self,ani,ax,p):
    if p=="save":
      ax.axis("equal")
      Writer = animation.ImageMagickFileWriter()
      ani.save('h.gif', writer = Writer)
    elif p=="save and show":
      ax.axis("equal")
      Writer = animation.ImageMagickFileWriter()
      ani.save('h.gif', writer = Writer)
      ax.axis("equal")
      plt.show()
    elif p=="show":
      ax.axis("equal")
      plt.show()
    else:
      print("不能识别的输入")
      return
 
  def run(self,excavator):
    '''
    实时运行
    '''
    def update(n):
      # 更新车辆位置
      excavator.setPosition(px[n],py[n])
      excavator,setPose(angle[n],0,0)
      excavator.getCurBody("radian")
      alpha = np.linspace(0,2*np.pi,100)
      x = excavator.position.x
      y = excavator.position.y
      xr = [self.check_range*np.cos(al)+x for al in alpha]
      yr = [self.check_range*np.sin(al)+y for al in alpha]
      p1.set_data(px[n],py[n])
      p2.set_data(excavator.up_outlines_x,excavator.up_outlines_y)
      p4.set_data(excavator.do_outlines_x,excavator.do_outlines_y)
      p3.set_data(xr,yr)
      t.set_text("x=%.3f,y=%.3f,yaw=%.3f/deg"%(excavator.position.x,excavator.position.y, np.rad2deg(excavator.pose.yaw)))
      # 显示检测范围内的电子围栏
      lines, dis = self.carRailDis(excavator)
      self.eWarn(excavator,lines,dis,None,warn,terminal=False,image=True) 
      return p1,p2,p3,t
    
    # 开始
    if self.test:
      print("开始测试 ...")
      # 创建图像                                  
      fig, ax, ax1, ax2, ax3, ax4, plt_yaw, plt_pitch, plt_roll = self.makeFigure()
      warn = ax1.text(0.02,0.2,'', fontsize=11, color="red")
      # 读取并显示围栏节点                        
      rail_lon, rail_lat = self.readRailPoints()
      rail = ClosedRail(rail_lon,rail_lat)
      rail.setForbidden("outside")
      self.addRail(rail)
      self.drawRail(ax)
      # 创建轨迹示例
      px,py,angle = excavator.pathGenerate()
      # 生成动图
      excavator.setPosition(px[0],py[0])
      excavator.setPose(angle[0],0,0)
      excavator.getCurBody('radian')
      p1,p2,p3,p4,t = self.drawExcavator(excavator, (ax, ax1, ax2, ax3, ax4), "plot")
      # 更新位姿并显示车辆
      ani = animation.FuncAnimation(fig,update,frames=len(px),interval=100,blit=True)
      # 保存图像为gif或显示动图
      self.endplot(ani,ax,"show")
    else:
      print("实时运行 ...")
      # 创建图像                                  
      fig, ax, ax1, ax2, ax3, ax4, plt_yaw, plt_pitch, plt_roll = self.makeFigure()
      warn = ax1.text(0.02,0.2,'', fontsize=11, color="red")

      # 读取并显示围栏节点
      self.readRails()
      self.drawRail(ax)

      # 创建接收节点
      rospy.init_node('e_rail_display', anonymous=False)
      pub = rospy.Publisher('warn',String,queue_size=1)
      print("ready to sub")
      if self.debug:
        rospy.Subscriber('pose2d',Pose2D,self.callback,excavator,queue_size=1,buff_size=52428800)
      else:
        if self.TF:
          tf_buffer = tf2_ros.Buffer()
          listener = tf2_ros.TransformListener(tf_buffer)
        else:
          rospy.Subscriber('/inspvax',Inspvax,self.callback,excavator,queue_size=1,buff_size=524248800)

  
      # 循环读取ROSTOPIC
      rate = rospy.Rate(100)
      start_time = time.time()
      
      i = 1
      while not rospy.is_shutdown():
        i += 1
        if self.TF:
          self.getCarPoints(tf_buffer,excavator)

        # 显示挖掘机位置
        if i==37:
          try:
            self.drawExcavator(excavator, (ax,ax1,ax2,ax3,ax4,p1,p2,p3,p4,t,plt_yaw,plt_pitch,plt_roll), "change")
          except Exception as e_change:
            rospy.loginfo(repr(e_change))
            try:
              p1,p2,p3,p4,t = self.drawExcavator(excavator, (ax,ax1,ax2,ax3,ax4), "plot")
            except Exception as e_plot:
              print(repr(e_plot))
          plt.draw()
          plt.pause(0.00000001)
          i = 1
        else:
          pass
        
        # 检测范围内的电子围栏计算距离
        lines, dis = self.carRailDis(excavator)

        # 发布消息
        pitch = excavator.pose.pitch
        roll = excavator.pose.roll
        x = excavator.position.x
        y = excavator.position.y
        angle = (pitch**2+roll**2)**0.5
        warnClass, angleClass = self.warningClass(x,y,dis,angle)
        pub.publish(str(warnClass)+','+str(angleClass))
        self.eWarn(excavator,lines,dis,None,warn,terminal=False,image=True) 
        
        # 输出报警记录
        dt=datetime.now()
        now0 = str(dt.strftime('%Y-%m-%d-%H'))
        now1 = str(dt.strftime('[%Y-%m-%d-%H:%M:%S]'))
        now2 = "["+str(rospy.Time.now())+"] "
        with open(os.environ['HOME']+"/e_rail_warn/eRail/log/warn_"+now0+".log",'a') as f:
          f.write(now1+now2+str(x)+','+str(y)+','+str(dis)+','+str(angle)+','+str(warnClass)+','+str(angleClass)+'\n')

        print((now1+now2+str(x)+','+str(y)+','+str(dis)+','+str(angle)+','+str(warnClass)+','+str(angleClass)))

        rate.sleep()

  def warningClass(self,x,y,dis,angle):
    if dis is None:
      warnClass = 0
    else:
      if dis <= 0.5 and dis > 0.3:
        warnClass = 1
      elif dis <= 0.3 and dis > 0.1:
        warnClass = 2
      elif dis <= 0.1:
        warnClass = 3
      else:
        warnClass = 0

    if angle >=20 and angle < 25:
      angleClass = 1
    elif angle >= 25 and angle < 30:
      angleClass = 2
    elif angle >= 30:
      angleClass = 3
    else:
      angleClass = 0

    return warnClass, angleClass


  def getCarPoints(self,tf_buffer,excavator):
    try:
      t = tf_buffer.lookup_transform('map','up_part',rospy.Time(0))
      x,y = self.transform2Position2D(t)
      excavator.setPosition(x,y)
      roll, pitch, yaw = self.transform2Pose3D(t)
      if yaw <0:
        yaw += 360
      excavator.setPose(yaw,pitch,roll)
  
      excavator.up_outlines_x = list()
      excavator.up_outlines_y = list()
      excavator.do_outlines_x = list()
      excavator.do_outlines_y = list()
      
      t = tf_buffer.lookup_transform('map','up1', rospy.Time(0))
      x,y = self.transform2Position2D(t)
      excavator.up_outlines_x.append(x)
      excavator.up_outlines_y.append(y)
      
      t = tf_buffer.lookup_transform('map','up2', rospy.Time(0))
      x,y = self.transform2Position2D(t)
      excavator.up_outlines_x.append(x)
      excavator.up_outlines_y.append(y)
      
      t = tf_buffer.lookup_transform('map','up3', rospy.Time(0))
      x,y = self.transform2Position2D(t)
      excavator.up_outlines_x.append(x)
      excavator.up_outlines_y.append(y)
      
      t = tf_buffer.lookup_transform('map','up4', rospy.Time(0))
      x,y = self.transform2Position2D(t)
      excavator.up_outlines_x.append(x)
      excavator.up_outlines_y.append(y)
      
      t = tf_buffer.lookup_transform('map','up5', rospy.Time(0))
      x,y = self.transform2Position2D(t)
      excavator.up_outlines_x.append(x)
      excavator.up_outlines_y.append(y)
      
      t = tf_buffer.lookup_transform('map','up6', rospy.Time(0))
      x,y = self.transform2Position2D(t)
      excavator.up_outlines_x.append(x)
      excavator.up_outlines_y.append(y)
      
      t = tf_buffer.lookup_transform('map','down1', rospy.Time(0))
      x,y = self.transform2Position2D(t)
      excavator.do_outlines_x.append(x)
      excavator.do_outlines_y.append(y)
      
      t = tf_buffer.lookup_transform('map','down2', rospy.Time(0))
      x,y = self.transform2Position2D(t)
      excavator.do_outlines_x.append(x)
      excavator.do_outlines_y.append(y)
      
      t = tf_buffer.lookup_transform('map','down3', rospy.Time(0))
      x,y = self.transform2Position2D(t)
      excavator.do_outlines_x.append(x)
      excavator.do_outlines_y.append(y)
      
      t = tf_buffer.lookup_transform('map','down4', rospy.Time(0))
      x,y = self.transform2Position2D(t)
      excavator.do_outlines_x.append(x)
      excavator.do_outlines_y.append(y)
    
      excavator.up_outlines_x.append(excavator.up_outlines_x[0])
      excavator.up_outlines_y.append(excavator.up_outlines_y[0])
      excavator.do_outlines_x.append(excavator.do_outlines_x[0])
      excavator.do_outlines_y.append(excavator.do_outlines_y[0])

    except Exception as e:
      rospy.loginfo(repr(e))
  
  def transform2Pose3D(self,tran):
    q = [tran.transform.rotation.x, tran.transform.rotation.y, tran.transform.rotation.z, tran.transform.rotation.w]
    euler = np.degrees(euler_from_quaternion(q))
    return euler[0], euler[1], euler[2]

  def transform2Position2D(self,tran):
    x, y = tran.transform.translation.x, tran.transform.translation.y
    return x,y

  def callback(self,data,excavator):
    if self.debug:
      x = data.x
      y = data.y
      a = data.theta
      # 当前车辆位置轮廓
      excavator.getCurBody(mode="radian")
    else:
      lon = data.longitude
      lat = data.latitude
      yaw = data.azimuth
      pitch = data.pitch
      roll = data.roll
      # 使用utm坐标系
      if self.utm:
        # 转化为utm坐标
        utm_point_x, utm_point_y = w2u(lon,lat)

        # 如果没有电子围栏，设置挖掘机位置的初始值为坐标原点
        if self.origin_x is None:
          self.origin_x, self.origin_y = utm_point_x, utm_point_y
        # 设置挖掘机位姿
        excavator.setPosition(utm_point_x,utm_point_y)
        excavator.setPose(yaw,pitch,roll)
        excavator.getCurBody(mode="degree",gps=False,originx=utm_point_x,originy=utm_point_y)
      
      # 使用经纬度
      else:
        # 如果没有电子围栏，设置挖掘机位置的初始值为坐标原点
        if self.origin_x is None:
          self.origin_x, self.origin_y = lon, lat
        # 设置挖掘机位姿
        excavator.setPosition(lon,lat)
        excavator.setPose(yaw,pitch,roll)
        excavator.getCurBody(mode="degree",gps=True,originx=lon,originy=lat)

      # 保存数据
      if self.log:
        with open("~/e_rial_warn/eRail/data/log.log",'a') as f:
          dt=datetime.now()
          ti = str(dt.strftime( '[%Y-%m-%d-%H:%M:%S] '))
          f.write(ti+str(excavator.position.x)+','+str(excavator.position.y)+','+str(excavator.pose.yaw)+'\n')
 
  def eWarn(self,excavator,lines,mindis,ax,warn,terminal=True,image=True):
    # 是否需要在终端显示
    if terminal:
      print(
          'position:('+str(excavator.position.x)+
          ','+str(excavator.position.y)+'),dis='+str(mindis))
      if mindis is not None:
        if mindis < self.distance_limit:
          rospy.loginfo("警告，距离电子围栏只有"+str(mindis)+"米！")
    # 是否需要在图像上显示
    if image:
      if lines is not None and ax is not None:
        try:
          p_sr = self.drawSelectedRails(lines, (ax,p_sr), excavator, "change")
        except:
          p_sr = self.drawSelectedRails(lines, ax, excavator, "plot")
      # 距离判断，并警告
      if mindis is not None:
        if mindis < self.distance_limit:
          warn.set_text("WARNING: %.4f metres!"%(mindis))
        else:
          warn.set_text('')
      else:
        warn.set_text('')


  def carRailDis(self,excavator):
    '''
    挖掘机与电子围栏距离的判断
    '''
    # 显示车辆检测范围内的电子围栏
    lines = self.selectRailRange(excavator)
    if len(lines) == 0:
      return None,None

    # 检测范围内电子围栏不为空时进行距离判断
    dis = list()
    # 上车体所有点的距离
    for i in range(len(excavator.up_outlines_x)-1):
    # for (x,y) in zip(excavator.up_outlines_x,excavator.up_outlines_y):
      x = excavator.up_outlines_x[i]
      y = excavator.up_outlines_y[i]
      xn = excavator.up_outlines_x[i+1]
      yn = excavator.up_outlines_y[i+1]
      for line in lines:
        for seg in line:
          Ax = seg[0][0]
          Ay = seg[1][0]
          Bx = seg[0][1]
          By = seg[1][1]
          if self.utm:
            dis.append(GIS.disBetweenPointAndSegment([x,y],[Ax,Ay],[Bx,By],'metre'))
            dis.append(GIS.disBetweenPointAndSegment([Ax,Ay],[x,y],[xn,yn],'metre'))
          else:
            dis.append(GIS.disBetweenPointAndSegment([x,y],[Ax,Ay],[Bx,By],'degree'))
            dis.append(GIS.disBetweenPointAndSegment([Ax,Ay],[x,y],[xn,yn],'degree'))
          # 判断如果上车体两点连线与电子围栏两点连线相交，则距离为0
          AV = Vector2d(Ax,Ay)
          BV = Vector2d(Bx,By)
          CV = Vector2d(x,y)
          DV = Vector2d(xn,yn)
          footA_CD = foot(CV,DV,AV)
          footB_CD = foot(CV,DV,BV)
          footC_AB = foot(AV,BV,CV)
          footD_AB = foot(AV,BV,DV)
          if (AV-footA_CD)*(BV-footB_CD)<0 and (CV-footC_AB)*(DV-footD_AB)<0:
            dis.append(0)

    # 下车体所有点的距离
    for i in range(len(excavator.do_outlines_x)-1):
    # for (x,y) in zip(excavator.do_outlines_x,excavator.do_outlines_y):
      x = excavator.do_outlines_x[i]
      y = excavator.do_outlines_y[i]
      xn = excavator.do_outlines_x[i+1]
      yn = excavator.do_outlines_y[i+1]
      for line in lines:
        for seg in line:
          Ax = seg[0][0]
          Ay = seg[1][0]
          Bx = seg[0][1]
          By = seg[1][1]
          if self.utm:
            dis.append(GIS.disBetweenPointAndSegment([x,y],[Ax,Ay],[Bx,By],'metre'))
            dis.append(GIS.disBetweenPointAndSegment([Ax,Ay],[x,y],[xn,yn],'metre'))
          else:
            dis.append(GIS.disBetweenPointAndSegment([x,y],[Ax,Ay],[Bx,By],'degree'))
            dis.append(GIS.disBetweenPointAndSegment([Ax,Ay],[x,y],[xn,yn],'degree'))
          # 判断如果上车体两点连线与电子围栏两点连线相交，则距离为0
          AV = Vector2d(Ax,Ay)
          BV = Vector2d(Bx,By)
          CV = Vector2d(x,y)
          DV = Vector2d(xn,yn)
          footA_CD = foot(CV,DV,AV)
          footB_CD = foot(CV,DV,BV)
          footC_AB = foot(AV,BV,CV)
          footD_AB = foot(AV,BV,DV)
          if (AV-footA_CD)*(BV-footB_CD)<0 and (CV-footC_AB)*(DV-footD_AB)<0:
            dis.append(0)
    
    if len(dis) > 0:
      return lines, min(dis)
    else:
      return lines, None

  def selectRailRange(self,excavator):
    rails = copy.deepcopy(self.rails)
    lines = list()
    for rail in rails:
      line = list()
      if rail.points_x is None or rail.points_y is None:
        continue
      for i in range(len(rail.points_x)-1):
        A = Vector2d(rail.points_x[i],rail.points_y[i])
        B = Vector2d(rail.points_x[i+1],rail.points_y[i+1])
        C = Vector2d(excavator.position.x,excavator.position.y)

        if self.utm:
          '''
          电子围栏为UTM坐标，挖掘机位置也为UTM坐标
          '''
          E,F = circleSegmentCrosses(A,B,C,self.check_range)
          pass
        else:
          '''
          电子围栏为经纬度坐标，A,B,C点也为经纬度坐标，
          将其转换为局部米坐标，返回线段AB与圆C的交点
          E,F，也为局部米坐标
          '''
          ox, oy = self.origin_x, self.origin_y
          ALx, ALy = GIS.geographyToLocal(A.x, A.y, ox, oy)
          BLx, BLy = GIS.geographyToLocal(B.x, B.y, ox, oy)
          CLx, CLy = GIS.geographyToLocal(C.x, C.y, ox, oy)
          
          AL = Vector2d(ALx, ALy)
          BL = Vector2d(BLx, BLy)
          CL = Vector2d(CLx, CLy)
          
          EL,FL = circleSegmentCrosses(AL,BL,CL,self.check_range)
          if EL is None and FL is None:
            E,F = None,None
          else:
            Ex, Ey = GIS.localToGeography(EL.x, EL.y, ox, oy)
            E = Vector2d(Ex, Ey)
            Fx, Fy = GIS.localToGeography(FL.x, FL.y, ox, oy)
            F = Vector2d(Fx, Fy)
        
        seg = list()
        if E is not None and F is not None:
          seg.append([E.x,F.x])
          seg.append([E.y,F.y])
          line.append(seg)
      if len(line):
        lines.append(line)
    return lines

  def drawSelectedRails(self,lines,inputs,excavator,mode):
    x = list()
    y = list()
    
    if len(lines)==0:
      return

    if mode=='plot':
      ax = inputs
      p = list()
      for line in lines:
        for seg in line:
          x = seg[0]
          y = seg[1]
          P, = ax.plot(x,y,'r.-')
          p.append(P)
      return p
    elif mode=='change':
      ax = inputs[0]
      p = inputs[1]
      p = list()
      for line in lines:
        for seg in line:
          x = seg[0]
          y = seg[1]
          P, = ax.plot(x,y,'r.-')
          p.append(P)
    else:
      print("输入错误")
      return

def circleSegmentCrosses(A,B,C,r):
  ax,ay = A.x,A.y
  bx,by = B.x,B.y
  cx,cy = C.x,C.y
  AB = Vector2d(bx-ax,by-ay)
  CA = Vector2d(ax-cx,ay-cy)
  CB = Vector2d(bx-cx,by-cy)
  ca = CA.module()
  cb = CB.module()
  ab = AB.module()
  # A在内，B在内
  if ca <= r and cb <= r:
    return A,B
  # A在内，B在外
  if ca <= r and cb >  r:
    D = foot(A,B,C)
    base = np.sqrt(r**2 - (D-C).module()**2)
    return A,D+base*AB*(1.0/AB.module())
  # A在外，B在内
  if ca > r  and cb <= r:
    D = foot(A,B,C)
    base = np.sqrt(r**2 - (D-C).module()**2)
    return D-base/AB.module()*AB,B
  # A在外，B在外
  if ca > r  and  cb > r:
    D = foot(A,B,C)
    AD = D-A 
    CD = D-C
    BD = D-B
    ad = AD.module()
    cd = CD.module()
    bd = BD.module()
    # AB直线与圆相交
    if cd <= r:
      # 垂足在圆内
      if ad < ab and bd < ab:
        base = np.sqrt(r**2 - cd**2)
        unit = AB/AB.module()
        return D+unit*(-base), D+unit*(base)
      else:
        return None,None
    else:
      return None,None
  
def foot(A,B,C):
  '''
  计算点C在线段AB上的垂足
  '''
  ax,ay = A.x,A.y
  bx,by = B.x,B.y
  cx,cy = C.x,C.y
  AB = B-A
  AC = C-A
  AD = AC*AB/AB.module()*AB/AB.module()
  return A+AD


def main():
  carw, carl = 4, 8
  check, limit = 10, 3
  excavator = Excavator(carw,carl)
  erail = ERail(check,limit,test=False,debug=False,log=False,utm=True,tf=True)
  erail.run(excavator)

if __name__ == '__main__':
  main()
