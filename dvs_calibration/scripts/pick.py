#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from dvs_msgs.msg import ImageObjectPoints

import Tkinter
import tkMessageBox
import ImageTk
import Image as Imagepy

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
        self.imageObjectPointsPub = rospy.Publisher('out/image_object_points', ImageObjectPoints, queue_size=1)
        self.lastImageOjectPoints = False
        self.lastImage = False

        self.ignorePub = rospy.Publisher('out/ignore', String, queue_size=1)
        self.takePub = rospy.Publisher('out/take', String, queue_size=1)

        #some default size
        self.lastImageSize = (512,512)

        self.title = rospy.get_namespace() + ' - ' + rospy.get_name()

        self.isStereoSlave = bool(rospy.get_param('~stereoslave',False))
        rospy.loginfo("am i stereo? %d",self.isStereoSlave)

        self.root = Tkinter.Tk()

        self.root.wm_title(self.title)
        self.label = Tkinter.Label(self.root, text='last pattern')
        self.label.pack()
        self.imgtk = None

        self.labelWaitStr = Tkinter.StringVar()
        self.labelWait = Tkinter.Label(self.root, textvariable=self.labelWaitStr)
        self.labelWait.pack()

        if self.isStereoSlave is not True:
            self.addButton = Tkinter.Button(self.root, text='Add Pattern', command=self.takeData)
            self.addButton.pack()

            self.ignoreButton = Tkinter.Button(self.root, text='Ignore Pattern', command=self.ignoreData)
            self.ignoreButton.pack()

        self.clearImage()
        self.stopWaitingForDecision()

    def startListener(self):
        self.points_sub = rospy.Subscriber("in/image_object_points", \
                ImageObjectPoints, self.pointsCallback)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("in/image_pattern",Image,self.imageCallback)


        self.ignoreSub = rospy.Subscriber("in/ignore",String,self.ignoreCallback)
        self.takeSub = rospy.Subscriber("in/take",String,self.takeCallback)


    def ignoreCallback(self,msg):
        """
        in stereo setup, reset callback to sync both windows
        """
        rospy.loginfo("got reset callback on topic")

        #reset everything
        self.ignoreData()

    def takeCallback(self,msg):
        rospy.loginfo("got take callback on topic")

        #reset everything
        self.takeData()

    def imageCallback(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I got an image")


        if self.waitingForDecision is True:
            rospy.loginfo(rospy.get_caller_id() + "ignore image (waiting for decision)")
            return
        
        self.lastImage = data

        #if we already got some objects points, we are free to go
        if self.lastImageOjectPoints is not False:
            #now we have both, block until a decision is made
            self.nowWaitForDecision()

    def showImage(self):
        if self.lastImage is not False:
            try:
                image_bgr8 = self.bridge.imgmsg_to_cv2(self.lastImage, "bgr8")
                self.lastImageSize = image_bgr8.shape[:2]

            except CvBridgeError as e:
                print(e)

            cv2image = cv2.cvtColor(image_bgr8, cv2.COLOR_BGR2RGBA)

            im = Imagepy.fromarray(cv2image)
            #im = Imagepy.fromstring('RGB', cv2.GetSize(self.lastImage), self.lastImage.tostring())
            # Convert the Image object into a TkPhoto object
            self.imgtk = ImageTk.PhotoImage(image=im) 

            # Put it in the display window
            #Tkinter.Label(root, image=imgtk).pack() 
            self.label.config(image=self.imgtk)

            rospy.sleep(0.05)

    def clearImage(self):
        #create new image of size x
        im = Imagepy.new('RGB', self.lastImageSize, 'black')

        # Convert the Image object into a TkPhoto object
        self.imgtk = ImageTk.PhotoImage(image=im) 

        #show image
        self.label.config(image=self.imgtk)

        rospy.sleep(0.05)


    def pointsCallback(self,data):
        rospy.loginfo(rospy.get_caller_id() + "got points callback")

        if self.waitingForDecision is True:
            rospy.loginfo(rospy.get_caller_id() + "ignore points (waiting for decision)")
            return

        #check if time difference to image is too large
        #image should always come before points
        me = data.header.stamp
        diff = me - self.lastImage.header.stamp
        #usually about 0.001537061
        maxDiff = rospy.Duration.from_sec(0.01)
        if diff > maxDiff:
            print("time difference was to much, do not take it", diff.to_sec())
            self.ignoreData()
            return


        #store data
        self.lastImageOjectPoints = data

        #self.scatterplot(data.image_points)

        self.showImage()

    def nowWaitForDecision(self):
        self.waitingForDecision = True
        self.labelWaitStr.set('waiting for decision')
        self.addButton.config(state="normal")
        self.ignoreButton.config(state="normal")

    def stopWaitingForDecision(self):
        self.waitingForDecision = False

        self.addButton.config(state="disabled")
        self.ignoreButton.config(state="disabled")

        #reset to false, so wait until we got object points
        #followed by an image
        #then wait for decision
        self.lastImageOjectPoints = False
        self.labelWaitStr.set('waiting for pattern')

    def ignoreData(self):
        rospy.loginfo("ignore data")

        msg = String('take')
        self.ignorePub.publish(msg)

        self.clearImage()
        self.stopWaitingForDecision()


    def takeData(self):
        if self.waitingForDecision is False:
            rospy.loginfo("do not take data, not complete")
            return

        rospy.loginfo("take data and publish it")

        self.imageObjectPointsPub.publish(self.lastImageOjectPoints)

        msg = String('take')
        self.takePub.publish(msg)

        self.clearImage()
        self.stopWaitingForDecision()

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

    def run(self):
        #start main loop of tkinter
        self.root.mainloop()

def main():
    rospy.init_node('picker', anonymous=True)
    p = Picker()
    p.startListener()
    try:
        # spin() simply keeps python from exiting until this node is stopped
        p.run()
    except KeyboardInterrupt:
        print("shutting down")
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
