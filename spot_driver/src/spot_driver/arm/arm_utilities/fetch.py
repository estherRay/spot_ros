#!/usr/bin/env python2.7

"""Get an image and command Spot to walk up to the selected object in the image."""

import rospy
import roslib
#import time, sys
import numpy as np
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
import message_filters
#from cv_bridge import CvBridge, CvBridgeError
#from bosdyn.client.image import ImageClient

class RequestManager:
    def __init__(self):
        print("HEllo")
        self.R_subscriber = message_filters.Subscriber("/spot/camera/frontright/image", Image)
        self.L_subscriber = message_filters.Subscriber("/spot/camera/frontleft/image", Image)
        #self._side_by_side = None
        self.g_all_click = []
        # Subscriber, Take only 1 image
        
        print("Check2")
        # Synchronize images
        self.ts = message_filters.ApproximateTimeSynchronizer([self.R_subscriber, self.L_subscriber], 10, 0.1, True)
        print("Check3")
        self.ts.registerCallback(self.image_callback)       
        print("Check4")

    def image_callback(self, img_r, img_l):
        rospy.loginfo('Images received')
        try:
            # Convert images for processing
            #self.cv_r = self.bridge.imgmsg_to_cv2(img_r, "passthrough")
            
            self.cv_r = cv_bridge.imgmsg_to_cv2(img_r, "passthrough")
            self.cv_l = self.bridge.imgmsg_to_cv2(img_l, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # Flip images by 90 degrees
        self.cv_r = cv2.transpose(self.cv_r) #right
        self.cv_r = cv2.flip(self.cv_r, 1)
        self.cv_l = cv2.transpose(self.cv_l) #left
        self.cv_l = cv2.flip(self.cv_l, 1)

        # Show both images horizontal, side by side, to each other
        self.side = np.hstack([self.cv_r, self.cv_l])
        image_title = 'Click to an object to fetch'
        cv2.namedWindow(image_title)
        cv2.setMouseCallback(image_title, self.cv_mouse_callback)

        self.clone = self.side.copy()

        while True:
            cv2.imshow(image_title, self.clone)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                # Quit
                print('"q" pressed, exiting.')
                break
        print("self.g_all_click " + str(self.g_all_click) + " type " + str(type(self.g_all_click)))
        print("self.g_all_click, 1st click " + str(self.g_all_click[0][0]) + " " + + str(self.g_all_click[0][1]))

        # Check which one is xmin and xmax
        if self.g_all_click[0][0] < self.g_all_click[1][0]:
            self.xmin = self.g_all_click[0][0]
            self.xmax = self.g_all_click[1][0]
        else:
            self.xmin = self.g_all_click[1][0]
            self.xmax = self.g_all_click[0][0]
        # Check which one is ymin and ymax
        if self.g_all_click[0][1] < self.g_all_click[1][1]:
            self.ymin = self.g_all_click[0][1]
            self.ymax = self.g_all_click[1][1]
        else:
            self.ymin = self.g_all_click[1][1]
            self.ymax = self.g_all_click[0][1]

        # Figure out which image was clicked on
        width, height = self.clone.shape
        print("width " + str(width) + " height " + str(height))
        avg_width = width / 2
        if (self.xmin < avg_width and self.xmax > avg_width) or (self.xmin > avg_width and self.xmax < avg_width):
            rospy.loginfo('Invalid: select an object on only one image')
        #elif self.xmin < avg_width:
            # This is left image
            #self.img_source = self.image_client.list_image_sources()
        #  print(str(self.image_client.list_image_sources()))
            #print(str(SpotWrapper._image_client.get_image_from_sources(["frontright_depth_in_visual_frame"])[0]))
            #self.depth_source = SpotWrapper._image_client.get_image_from_sources(["frontright_depth_in_visual_frame"])[0]
    # else:
            # This is right image
            #self.img_source = 
            #self.depth_source = self._image_client.get_image_from_sources(["frontleft_depth_in_visual_frame"])[0]
            #return True

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

    
def fetch_main(self, spot_wrapper):
    #rospy.init_node('my_fetch', anonymous=True)
    #rospy.sleep(0.1)
    rospy.loginfo("Starting fetch")
    try:
        __init__(self, spot_wrapper)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
        cv2.destroyAllWindows()

"""
class fetch():

    def __init__(self):
        self.image_client = self.ensure_client(ImageClient.default_service_name)
        #self._side_by_side = None
        self.g_all_click = []
        # Subscriber, Take only 1 image
        self.R_subscriber = message_filters.Subscriber("/spot/camera/frontright/image", Image)
        self.L_subscriber = message_filters.Subscriber("/spot/camera/frontleft/image", Image)
        # Synchronize images
        self.ts = message_filters.ApproximateTimeSynchronizer([self.R_subscriber, self.L_subscriber], 10, 0.1, True)
        self.ts.registerCallback(self.image_callback)       
        

    def image_callback(self, img_r, img_l):
        rospy.loginfo('Images received')
        try:
            # Convert images for processing
            self.cv_r = self.bridge.imgmsg_to_cv2(img_r, "passthrough")
            self.cv_l = self.bridge.imgmsg_to_cv2(img_l, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # Flip images by 90 degrees
        self.cv_r = cv2.transpose(self.cv_r) #right
        self.cv_r = cv2.flip(self.cv_r, 1)
        self.cv_l = cv2.transpose(self.cv_l) #left
        self.cv_l = cv2.flip(self.cv_l, 1)

        # Show both images horizontal, side by side, to each other
        self.side = np.hstack([self.cv_r, self.cv_l])
        image_title = 'Click to an object to fetch'
        cv2.namedWindow(image_title)
        cv2.setMouseCallback(image_title, self.cv_mouse_callback)

        self.clone = self.side.copy()

        while True:
            cv2.imshow(image_title, self.clone)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                # Quit
                print('"q" pressed, exiting.')
                break
        print("self.g_all_click " + str(self.g_all_click) + " type " + str(type(self.g_all_click)))
        print("self.g_all_click, 1st click " + str(self.g_all_click[0][0]) + " " + + str(self.g_all_click[0][1]))

        # Check which one is xmin and xmax
        if self.g_all_click[0][0] < self.g_all_click[1][0]:
            self.xmin = self.g_all_click[0][0]
            self.xmax = self.g_all_click[1][0]
        else:
            self.xmin = self.g_all_click[1][0]
            self.xmax = self.g_all_click[0][0]
        # Check which one is ymin and ymax
        if self.g_all_click[0][1] < self.g_all_click[1][1]:
            self.ymin = self.g_all_click[0][1]
            self.ymax = self.g_all_click[1][1]
        else:
            self.ymin = self.g_all_click[1][1]
            self.ymax = self.g_all_click[0][1]

        # Figure out which image was clicked on
        width, height = self.clone.shape
        print("width " + str(width) + " height " + str(height))
        avg_width = width / 2
        if (self.xmin < avg_width and self.xmax > avg_width) or (self.xmin > avg_width and self.xmax < avg_width):
            rospy.loginfo('Invalid: select an object on only one image')
        elif self.xmin < avg_width:
            print(str())
            # This is left image
            #self.img_source = self.image_client.list_image_sources()
            print(str(self.image_client.list_image_sources()))
            #print(str(SpotWrapper._image_client.get_image_from_sources(["frontright_depth_in_visual_frame"])[0]))
            #self.depth_source = SpotWrapper._image_client.get_image_from_sources(["frontright_depth_in_visual_frame"])[0]
        else:
            # This is right image
            #self.img_source = 
            #self.depth_source = self._image_client.get_image_from_sources(["frontleft_depth_in_visual_frame"])[0]
            return True

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


    
    def fetch_main(self):
        rospy.init_node('my_fetch', anonymous=True)
        rospy.sleep(0.1)
        rospy.loginfo("Starting load_image_node node")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down ROS Image feature detector module"
            cv2.destroyAllWindows()
    

if __name__ == "__main__":
    class_fetch = fetch()
    class_fetch.fetch_main()
"""
