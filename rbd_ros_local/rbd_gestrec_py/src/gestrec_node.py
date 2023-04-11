#!/usr/bin/env python


### imports
import rospy
from std_msgs.msg import String
#from __future__ import print_function
#from rbd_gestrec_py.srv import getLastGesture, getLastGestureResponse
from rbd_msgs.srv import GetLastGesture, GetLastGestureResponse


### classes
class gestrec():
    def __init__(self):
        rospy.loginfo("Gesture recognition node started")

        ### global variables
        self.last_gesture = "none"

    ### continuously get last gesture
    def callback(self, data):
        self.last_gesture = data.data
        rospy.loginfo("received gesture ", self.last_gesture)

    def listener(self):
        rospy.Subscriber("chatter", data_class=String, callback=self.callback)


    ### hand out last gesture using a service
    def handle_get_last_gesture(self, req):
        # wait for gesture
        self.last_gesture = "none"
        while(self.last_gesture == "none"):
            x = 0    
        return GetLastGestureResponse(self.last_gesture)

    def get_last_gesture_server(self):
        s = rospy.Service('get_last_gesture', GetLastGesture, self.handle_get_last_gesture)


### main
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    myGestrec = gestrec()
    myGestrec.listener()
    myGestrec.get_last_gesture_server()

    rospy.spin()

