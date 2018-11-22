#!/usr/bin/env python

import socket
import rospy
import sys
from nmea_msgs.msg import Sentence


def nmea_collector():

    rospy.init_node('NMEA_fix')

    client = setup_connection()

    pub = rospy.Publisher('nmea_fix_sentence', Sentence, queue_size=10)

    nmea_sentence = Sentence()

    rate = rospy.Rate(100)
    rospy.loginfo("Starting data collection...")
    while not rospy.is_shutdown():
        data = list(client.recv(1024).split('\r\n'))
        nmea_sentence.sentence = data[0]
        rospy.loginfo(nmea_sentence)
        pub.publish(nmea_sentence)
        rate.sleep()

    rospy.spin()


def setup_connection():
    port = 4567
    port = 53
    attempts_limit = 10
    current_attempt = 0
    connected = False
    ip = None

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    while not connected and not rospy.is_shutdown() and current_attempt < attempts_limit:
        current_attempt += 1

        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.settimeout(5)
            ip = socket.gethostbyname("192.168.1.100")
	    ip = socket.gethostbyname("192.168.43.114")
            address = (ip, port)

            rospy.loginfo("Attempting connection to %s:%s ", ip, port)
            client.connect(address)

            rospy.loginfo("=====================================")
            rospy.loginfo("Connected to %s:%s ", ip, port)
            rospy.loginfo("=====================================")
            connected = True
        except Exception as e:
            rospy.logwarn("Connection to IP: " + ip + ": " + e.__str__() +
                          ".\nRetrying connection: Attempt: %s/%s",
                          current_attempt, attempts_limit)

    if not connected:
        rospy.logerr("No connection established. Node shutting down")
        sys.exit()

    return client


if __name__ == '__main__':
    try:
        nmea_collector()
    except rospy.ROSInterruptException:
        pass
