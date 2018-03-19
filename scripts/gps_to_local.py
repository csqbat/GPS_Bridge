#!/usr/bin/env python


import rospy, math, tf
import numpy as np
import sys, termios, tty, select, os
from dji_sdk.msg import GlobalPosition
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# lots of help from: https://www.movable-type.co.uk/scripts/latlong.html
# dji info http://wiki.ros.org/dji_sdk


def toRad(num):
  return 0.017453292519943 * float(num)

def toDeg(num):
  return 57.295779513082 * float(num)


def GPS_distance(lat1, lon1, lat2, lon2):
  ### Distance in meters from [lat1, lon1] to [lat2, lon2]
  R = 6371000.0
  lat1 = toRad(lat1)
  lat2 = toRad(lat2)

  delta_lat = lat2-lat1
  delta_lon = toRad(lon2-lon1)
  a = math.sin(delta_lat/2.0) * math.sin(delta_lat/2.0) + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon/2.0) * math.sin(delta_lon/2.0)
  c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0-a))
  d = R * c
  return d

def GPS_point_at_dist_bearing(lat1, lon1, d, brng):
  # Gives location of [lat2, lon2] which is d meters away at a bearing of b from [lat1, lon1]
  R = 6371000.0
  lat1 = toRad(lat1)
  lon1 = toRad(lon1)
  
  lat2 = math.asin( math.sin(lat1)*math.cos(d/R) + math.cos(lat1)*math.sin(d/R)*math.cos(brng) )
  lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat1), math.cos(d/R)-math.sin(lat1)*math.sin(lat2))
  return [toDeg(lat2), toDeg(lon2)]

def GPS_bearing(lat1, lon1, lat2, lon2):
  # Bearing from [lat1 lon1] to [lat2, lon2] in degrees
  lat1 = toRad(lat1)
  lon1 = toRad(lon1)
  lat2 = toRad(lat2)
  lon2 = toRad(lon2)

  y = math.sin(lon2-lon1) * math.cos(lat2)
  x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(lon2-lon1)
  brng = math.atan2(y, x)
  return toDeg(brng)

def GPS_to_XY(lat1, lon1, lat2, lon2):
  d = GPS_distance(lat1, lon1, lat2, lon2)
  b = GPS_bearing(lat1, lon1, lat2, lon2)

  y = d * -math.cos(toRad(b))
  x = d * math.sin(toRad(b))
  return [x, y]

class GPS_to_Local(object):
  # Class that handles callbacks and subscribers
  origin_lat = -1.0
  origin_lon = -1.0
  ref_lat = -1.0
  ref_lon = -1.0
  initialized = False

  def init(self, origin_lat, origin_lon):
    ### Set the origin point
    self.origin_lat = origin_lat
    self.origin_lon = origin_lon

    # Setup publisher
    self.pub_odom = rospy.Publisher('/global/odom', Odometry, queue_size=1)
    # Setup Subs
    # get true odom from the quad
    self.sub_odom = rospy.Subscriber("/dji_sdk/odometry", Odometry, self.callback_odom, queue_size=1)
    self.sub_gps = rospy.Subscriber("/dji_sdk/global_position", GlobalPosition, self.callback_gps, queue_size=1)
    self.pub_tf = tf.TransformBroadcaster()
    self.good_odom = False
    
#    # create a test point 1,000 m south-and 1,000 m east of me
#    dist = 1414.21356237
#    brng = toRad(135.0)
#    [self.ref_lat, self.ref_lon] = GPS_point_at_dist_bearing(self.origin_lat, self.origin_lon, dist, brng)
#    d2 = GPS_distance(self.ref_lat, self.ref_lon, self.origin_lat, self.origin_lon)
#    b2 = GPS_bearing(self.origin_lat, self.origin_lon, self.ref_lat, self.ref_lon)
#    rospy.loginfo("Ref: %0.5f / %0.5f", self.ref_lat, self.ref_lon)
#    rospy.loginfo("     Dist / Bearing: %0.5f / %0.5f", d2, b2)
#    rospy.loginfo("     Origin: %0.5f / %0.5f", self.origin_lat, self.origin_lon)
#    [x, y] = GPS_to_XY(self.origin_lat, self.origin_lon, self.ref_lat, self.ref_lon)
#    rospy.loginfo("     x/y: %0.2f / %0.2f", x,y)
  

  def callback_gps(self, gps_msg):
    if self.good_odom:# and gps_msg.health >= 2:
      [x, y] = GPS_to_XY(self.origin_lat, self.origin_lon, gps_msg.latitude, gps_msg.longitude)
      #rospy.logerr("x %.2f and y %.2f", x ,y)
      odom = self.odom #Trust values from odom
      odom.pose.pose.position.x = x # except x and y, which I want in my own frame which is relative to a specified origin
      odom.pose.pose.position.y = -y
      self.pub_odom.publish(odom)   
      self.pub_tf.sendTransform((x, -y, odom.pose.pose.position.z), (odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w),rospy.Time.now(), "/global_quad", "/world")
    
  def callback_odom(self, odom_msg):
      self.good_odom = True
      euler = tf.transformations.euler_from_quaternion((odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w))
      roll = euler[0]
      pitch = euler[1]
      yaw = -euler[2] + math.pi/2.0
      [odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w] = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
      #rospy.logwarn("rpy: %0.2f, %0.2f, %0.2f", roll, pitch, yaw)
      self.odom = odom_msg
      

if __name__ == '__main__':
  rospy.init_node('GPS_to_Local')
  north_lat = rospy.get_param('/north_lat')
  west_lon = rospy.get_param('/west_lon')
  south_lat = rospy.get_param('/south_lat')
  east_lon = rospy.get_param('/east_lon')

  origin_lat = (float(north_lat) + float(south_lat))/2.0
  origin_lon = (float(west_lon) + float(east_lon))/2.0

  rospy.loginfo("GPS_to_Local::initializing")
  rospy.loginfo(" GPS_to_Local::origin_lat: %0.5f", origin_lat)
  rospy.loginfo(" GPS_to_Local::origin_lon: %0.5f", origin_lon)

  bridge = GPS_to_Local()
  bridge.init(origin_lat, origin_lon)
  rospy.loginfo("GPS_to_Local::initialized")
  rospy.spin()
