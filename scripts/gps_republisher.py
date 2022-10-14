#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


class GPSRepublisher:
    """
    Simple republisher for GPS signals. The idea is to (de)activate GPS data reception in certain states.
    """

    def __init__(self):
        self.publish_gps = True
        # the new source of GPS data the robot is subscribed to
        self.gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=1)
        # the original source of GPS data to be forwarded to the robot
        rospy.Subscriber('/fix', NavSatFix, self.gps_fix_callback, queue_size=1)
        # triggers GPS (de)activation
        rospy.Subscriber('/toggle_gps', String, self.toggle_gps, queue_size=1)

    def toggle_gps(self, msg):
        """
        (De)activates GPS signal forwarding.

        @param msg: callback message
        """
        if self.publish_gps:
            print("deactivating GPS signal forwarding..")
        else:
            print("activating GPS signal forwarding..")
        self.publish_gps = not self.publish_gps

    def gps_fix_callback(self, msg):
        """
        Republishes the GPS signal under the new topic (if republishing is activated).

        @param msg: GPS signal to be forwarded
        """
        if self.publish_gps:
            print("publishing gps")
            self.gps_pub.publish(msg)


def node():
    rospy.init_node('gps_republisher')
    GPSRepublisher()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
