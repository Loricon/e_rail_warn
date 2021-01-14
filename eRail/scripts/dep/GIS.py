# -*- coding:UTF-8 -*-
import numpy as np

global R_lon
R_lon = 6356755 #(m)
global R_lat0
R_lat0 = 6378140 #(m)

def geographyToLocal(lon,lat,or_lon,or_lat):
  global R_lon
  global R_lat0
  R_lat = np.cos(np.deg2rad(or_lat))*R_lat0
  x = np.deg2rad(lon-or_lon)*R_lat
  y = np.deg2rad(lat-or_lat)*R_lon
  return x,y

def localToGeography(x,y,or_lon,or_lat):
  global R_lon
  global R_lat0
  R_lat = np.cos(np.deg2rad(or_lat))*R_lat0
  lon = np.rad2deg(x/R_lat) + or_lon
  lat = np.rad2deg(y/R_lon) + or_lat
  return lon,lat

def latToY(lat,or_lon,or_lat):
  global R_lon
  global R_lat0
  y = np.deg2rad(lat-or_lat)*R_lon
  return y

def lonToX(lon,or_lon,or_lat):
  global R_lon
  global R_lat0
  R_lat = np.cos(np.deg2rad(or_lat))*R_lat0
  x = np.deg2rad(lon-or_lon)*R_lat
  return x

def yToLat(y,or_lon,or_lat):
  global R_lon
  global R_lat0
  lat = np.rad2deg(y/R_lon) + or_lat
  return lat

def xToLon(x,or_lon,or_lat):
  global R_lon
  global R_lat0
  R_lat = np.cos(np.deg2rad(or_lat))*R_lat0
  lon = np.rad2deg(x/R_lat) + or_lon
  return lon

def disBetweenPointAndLine(point,line,mode="metre"):
  dis = list()
  if mode=="degree":
    '''
    如果输入是经纬度的话，先转换成局部坐标，以米为单位
    '''
    px,py = geographyToLocal(point[0],point[1],line[0][0],line[0][1])
    R = list()
    for l in line:
      rx,ry = geographyToLocal(l[0],l[1],line[0][0],line[0][1])
      R.append([rx,ry])
    
    for i in range(len(line-1)):
      dis.append(disBetweenPointAndSegment([px,py],R[i],R[i+1]))

  elif mode=="metre":
    '''
    如果输入是以米为单位的局部坐标的话，直接计算
    '''
    for i in range(len(line)-1):
      dis.append(disBetweenPointAndSegment(point,line[i],line[i+1]))
  else:
    print("模式不能识别")
    return

  return min(dis)


def disBetweenPointAndSegment(P,A,B,mode):
  # 如果以经纬度输入，则转化成以米为单位的局部坐标
  if mode=="degree":
    Am = geographyToLocal(A[0],A[1],P[0],P[1])
    Bm = geographyToLocal(B[0],B[1],P[0],P[1])
    Pm = geographyToLocal(P[0],P[1],P[0],P[1])
    v1 = [Bm[0]-Am[0],Bm[1]-Am[1]]
    v2 = [Pm[0]-Am[0],Pm[1]-Am[1]]
    v3 = [Pm[0]-Bm[0],Pm[1]-Bm[1]]
  elif mode=="metre":
    v1 = [B[0]-A[0],B[1]-A[1]]
    v2 = [P[0]-A[0],P[1]-A[1]]
    v3 = [P[0]-B[0],P[1]-B[1]]
  # 距离
  if abs(v1[0])<10e-10 and abs(v1[1])<10e-10:
    return np.sqrt(v2[0]**2 + v2[1]**2)
  elif np.dot(v1,v2)<0 and np.dot(v1,v3)<0:
    return np.sqrt(v2[0]**2 + v2[1]**2)
  elif np.dot(v1,v2)>0 and np.dot(v1,v3)>0:
    return np.sqrt(v3[0]**2 + v3[1]**2)
  elif np.dot(v1,v2)>0 and np.dot(v1,v3)<0:
    return np.linalg.norm(np.cross(v1,v2))/np.linalg.norm(v1)
  else:
    print("Point segment distance error")
    return


