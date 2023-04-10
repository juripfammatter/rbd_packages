#!/usr/bin/env python


### imports
import rospy
from std_msgs.msg import String
#from __future__ import print_function
from rbd_gestrec_py.srv import getLastGesture, getLastGestureResponse


### classes
class gestrec():
    def __init__(self):
        print("package works")

        ### global variables
        self.last_gesture = "none"

    ### continuously get last gesture
    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.last_gesture = data.data
        print("Saved this in global variable: ", self.last_gesture)

    def listener(self):
        print("Enter listener()")
        rospy.Subscriber("chatter", data_class=String, callback=self.callback)


    ### hand out last gesture using a service
    def handle_get_last_gesture(self, req):
        print("Returning [%s]"%(self.last_gesture))

        # wait for gesture
        self.last_gesture = "none"
        while(self.last_gesture == "none"):
            print("waiting for gesture")

        return getLastGestureResponse(self.last_gesture)

    def get_last_gesture_server(self):
        print("Enter get_last_gesture_server()")
        s = rospy.Service('get_last_gesture', getLastGesture, self.handle_get_last_gesture)
        print("Ready to return last gesture.")


### main
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    print("Hello World\n")
    myGestrec = gestrec()
    myGestrec.listener()
    myGestrec.get_last_gesture_server()

    rospy.spin()

