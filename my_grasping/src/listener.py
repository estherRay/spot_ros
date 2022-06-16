#!/usr/bin/env python2.7

"""Receive pixel info"""

import rospy
import roslib
import time, sys
from my_grasping.msg import PixelPose
from spot_driver.spot_wrapper import walk_to_object_image


class pixel_receive():

  def __init__(self):
    self.global_pixel = None


  def callback(self, obj_point):
    rospy.loginfo('Obj loc received')
    self.global_pixel = obj_point.value
    print(type(self.global_pixel))
    print(str(self.global_pixel))
    self.walk_to_object_image(object_point = self.global_pixel)


  def main(self):
    print("Hello")
    rospy.init_node('pixel_receive', anonymous=True)
    self.pixel_loc = rospy.Subscriber('object_location', PixelPose, self.callback, queue_size=10)
    print("Here")
    rospy.spin()

if __name__ == "__main__":
  pixr = pixel_receive()
  pixr.main()