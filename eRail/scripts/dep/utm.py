#!/usr/bin/env python

from geodesy.utm import *

def wgs84_utm_conversion_3d(lon,lat,alt):
    # UTM point in Pickle Research Campus, University of Texas, Austin
    ll = GeoPoint(latitude = lat,longitude = lon,altitude = alt)
    pt = fromMsg(ll)
    point_xyz = pt.toPoint()
    return point_xyz.x,point_xyz.y,point_xyz.z

def wgs84_utm_conversion_2d(lon,lat):
    # same point, but without altitude
    pt = fromLatLong(lat, lon)
    point_xy = pt.toPoint()
    return point_xy.x,point_xy.y
 
if __name__ == '__main__':
    lat = 30.385315
    lon = 97.728524
    alt = 209.0

    wgs84_utm_conversion_3d(lon,lat,alt)
    wgs84_utm_conversion_2d(lon,lat)

    
