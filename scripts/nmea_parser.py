#!/usr/bin/env python

import socket
import rospy
import sys
from nmea_msgs.msg import Sentence


def nmea_collector(ip , port):

    client = setup_connection(ip , port)

    main_antenna_GGA_pub = rospy.Publisher('nmea/main_GGA', Sentence, queue_size=10)
    secondary_antenna_GGA_pub = rospy.Publisher('nmea/secondary_GGA', Sentence, queue_size=10)
    HRP_pub = rospy.Publisher('nmea/HRP', Sentence, queue_size=10)

    main_GGA_sentence = Sentence()
    HRP_sentence = Sentence()
    secondary_GGA_sentence = Sentence()

    rate = rospy.Rate(100)
    rospy.loginfo("Starting data retrieval...")

    while not rospy.is_shutdown():
        GPS_data = list(client.recv(1024).split('\r\n'))

        main_GGA_sentence.sentence = GPS_data[0]
        HRP_sentence.sentence = GPS_data[1]
        secondary_GGA_sentence.sentence = GPS_data[2]

        main_antenna_GGA_pub.publish(main_GGA_sentence)
        HRP_pub.publish(HRP_sentence)
        secondary_antenna_GGA_pub.publish(secondary_GGA_sentence)

        rate.sleep()

    rospy.spin()


def setup_connection(_ip, _port):
    port = _port
    ip = None
    attempts_limit = 10
    current_attempt = 0
    connected = False

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    while not connected and not rospy.is_shutdown() and current_attempt < attempts_limit:
        current_attempt += 1

        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.settimeout(5)
            ip = socket.gethostbyname(_ip)
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
        rospy.init_node('NMEA_parser')
        if not(2 < len(sys.argv) <= 3) or not str(sys.argv[1]).__contains__("ip:") or not str(sys.argv[2]).__contains__("port:"):
            rospy.logwarn("Incorrect passing of arguments. Please try again using the following format: \n"
                          "rosrun nmea_fix nmea_fix.py ip:xxx.xxx.xxx.xxx port:xxxxxx")
        else:
            nmea_collector(str(sys.argv[1]).split(":")[1], int(str(sys.argv[2]).split(":")[1]))
    except rospy.ROSInterruptException:
        pass
