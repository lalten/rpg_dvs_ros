#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from dvs_msgs.msg import ImageObjectPoints

import Tkinter
import tkMessageBox

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Picker(object):
    """
    pick messages out of topic, which should be forwarded

    user will be ask every time a new message arrives, if he wants to take it
    the user can verify, that the LED pattern was correctly detected
    and prevent wrong calibration results caused by wrong patterns
    """
    def __init__(self):
        self.imageObjectPointsPub = rospy.Publisher('out/image_object_points', ImageObjectPoints, queue_size=10)
        self.lastImageOjectPoints = ImageObjectPoints()
        self.lastImage = False

        self.title = rospy.get_namespace() + ' - ' + rospy.get_name()

        cv2.namedWindow(self.title, 1)

    def startListener(self):
        self.points_sub = rospy.Subscriber("in/image_object_points", \
                ImageObjectPoints, self.pointsCallback)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("in/image_pattern",Image,self.imageCallback)


    def imageCallback(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I got an image")
        try:
            self.lastImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


    def pointsCallback(self,data):
        self.lastImageOjectPoints = data
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.image_points)
        #self.scatterplot(data.image_points)

        #show last image
        cv2.imshow(self.title, self.lastImage)
        #do not wait at all
        cv2.waitKey(1)

        #ask if we want to take that pattern
        self.askTakeIt()

    def takeData(self):
        rospy.loginfo("take data and publish it")
        self.imageObjectPointsPub.publish(self.lastImageOjectPoints)

    def askTakeIt(self):
        answer = tkMessageBox.askyesno(self.title, 'Do you want to use this detected pattern for calibration?')
        if answer:
            self.takeData()
            #tkMessageBox.showinfo('No', 'Quit has been cancelled')

    def scatterplot(self,points):
        import matplotlib.pyplot as plt
        import random
        import numpy as np

        #in images ---x-->
        # and y downwards
        #     |
        #    \/

        x = []
        y = []
        for i in points:
            x.append(i.x)
            y.append(128-i.y)

        plt.axis((0,128,0,128))

        #get random color between 0 and 1
        #and use it for all points
        #TODO: not working yet
        colors = (100 * random.random()) * np.ones(len(x))

        plt.title(self.title)
        plt.scatter(x, y, c=colors, alpha=0.5)
        
        #make interactive plot, so it is nonblocking
        plt.ion()

        #show data witout clearing image
        #so we add up the examples
        plt.show()

def main():
    rospy.init_node('picker', anonymous=True)
    p = Picker()
    p.startListener()
    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
