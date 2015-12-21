#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from dvs_msgs.msg import ImageObjectPoints

import Tkinter
import tkMessageBox



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
        self.lastPattern = 0

    def startListener(self):
        rospy.Subscriber("in/image_object_points", ImageObjectPoints, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def callback(self,data):
        self.lastImageOjectPoints = data
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.image_points)
        self.scatterplot(data.image_points)
        self.askTakeIt()

    def takeData(self):
        rospy.loginfo("take data and publish it")
        self.imageObjectPointsPub.publish(self.lastImageOjectPoints)

    def askTakeIt(self):
        answer = tkMessageBox.askyesno('Verify Pattern', 'Do you want to use this detected pattern for calibration?')
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

        print("colors are",colors)
        plt.scatter(x, y, c=colors, alpha=0.5)
        
        #make interactive plot, so it is nonblocking
        plt.ion()

        #show data witout clearing image
        #so we add up the examples
        plt.show()

if __name__ == '__main__':
    try:
        #talker()
        rospy.init_node('picker', anonymous=True)
        p = Picker()
        p.startListener()
    except rospy.ROSInterruptException:
        pass
