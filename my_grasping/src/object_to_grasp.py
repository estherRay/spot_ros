#!/usr/bin/env python3

"""Get an image from the gripper camera of Spot. User inputs a pixel on the object to be picked up.
    The pixel information is sent as PixelList to spot_ros, then spot_wrapper to command Spot to walk up to the selected object in the image."""

import rospy
import roslib
import time, sys
from sensor_msgs.msg import Image
from my_grasping.msg import PixelList
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty


class image_display():

  def __init__(self):
    self.bridge = CvBridge()
    self.g_image_click = None
    self.g_all_click = []
    self.g_image_display = None

    # Subscriber, Take only 1 image
    self.image_subscriber = rospy.Subscriber("/spot/camera/hand_color/image", Image, self.image_callback, queue_size=10)

  def image_callback(self, img_msg):
    rospy.loginfo('Image received')
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
      rospy.logerr("CvBridge Error: {0}".format(e))

    # Show image to user and wait for them to click on a pixel
    rospy.loginfo('Click on an object to walk up to')
    image_title = 'Click to walk up to something'
    cv2.namedWindow(image_title)
    cv2.setMouseCallback(image_title, self.cv_mouse_callback)
    
    self.g_image_display = self.cv_image
    self.clone = self.g_image_display.copy()
    
    while True:
        cv2.imshow(image_title, self.clone)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q'):
            # Quit
            print('"q" pressed, exiting.')
            break
    
    # Publishing location of the object in the image

    print("self.g_all_click " + str(self.g_all_click) + " type " + str(type(self.g_all_click)))# + " " + str(self.g_all_click[1].x))
    print("self.g_all_click, 1st click x " + str(self.g_all_click[0][0]))

    my_points = PixelList()
    # Check which one is xmin and xmax
    if self.g_all_click[0][0] < self.g_all_click[1][0]:
        my_points.xmin = self.g_all_click[0][0]
        my_points.xmax = self.g_all_click[1][0]
    else:
        my_points.xmin = self.g_all_click[1][0]
        my_points.xmax = self.g_all_click[0][0]
    # Check which one is ymin and ymax
    if self.g_all_click[0][1] < self.g_all_click[1][1]:
        my_points.ymin = self.g_all_click[0][1]
        my_points.ymax = self.g_all_click[1][1]
    else:
        my_points.ymin = self.g_all_click[1][1]
        my_points.ymax = self.g_all_click[0][1]

    self.location_publisher.publish(my_points)


  def cv_mouse_callback(self, event, x, y, flags, param):
    # when mouse click, take first corner
    if event == cv2.EVENT_LBUTTONDOWN:
        #self.g_image_click = (x, y)
        self.g_all_click = [(x, y)]
    # when mouse released, take second corner
    elif event == cv2.EVENT_LBUTTONUP:
        self.g_all_click.append((x, y))
        cv2.rectangle(self.clone, self.g_all_click[0], self.g_all_click[1], (0,255,0), 2)
        #cv2.imshow("Click to make a box", self.clone)


  def main(self):
    rospy.init_node('image_display', anonymous=True)
    self.location_publisher = rospy.Publisher('object_to_grasp', PixelList, queue_size=10)
    
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print ("Shutting down ROS Image feature detector module")
      cv2.destroyAllWindows()

if __name__ == "__main__":
  imdis = image_display()
  imdis.main()