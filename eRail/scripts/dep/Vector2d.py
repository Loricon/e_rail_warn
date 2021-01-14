#-*- coding: UTF-8 -*- 

import numpy as np
import numbers as n
class Vector2d:
  x = 0
  y = 0
  
  def __init__(self,x,y):
    self.x = x
    self.y = y

  def __add__(self,v):
    return Vector2d(self.x+v.x, self.y+v.y)

  def __sub__(self,v):
    return Vector2d(self.x-v.x, self.y-v.y)

  def __mul__(self,v):
    if isinstance(v,n.Number):
      return Vector2d(self.x*v, self.y*v)
    elif type(v) == type(self):
      return self.x*v.x + self.y*v.y
  
  def __rmul__(self,v):
    if isinstance(v,n.Number):
      return Vector2d(self.x*v, self.y*v)
    elif type(v) == type(self):
      return self.x*v.x + self.y*v.y

  def __truediv__(self,a):
    if isinstance(a,n.Number):
      if abs(1.0*a) < 10e-8:
        print("Vector2d divided by zero")
        return None
      else:
        return Vector2d(self.x/a, self.y/a)
    else:
      print("Vector2d divided by error type")
      return None
  
  def module(self):
    return np.sqrt(self.x**2 + self.y**2)

  def show(self):
    print("<"+str(self.x)+","+str(self.y)+">")
