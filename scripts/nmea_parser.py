#!/usr/bin/env python

import socket
import rospy
import sys
from nmea_msgs.msg import Sentence


def create_publishers(GPS_data):
    publishers = []
    for i in range(0,len(GPS_data)):
        if(str(GPS_data[i]).startswith("$GP")):
            pub = rospy.Publisher(
                                "nmea/" + str(GPS_data[i]).split(',')[0][3:len(str(GPS_data[i]).split(',')[0])],
                                Sentence,
                                queue_size=10
                                )
            publishers.append(pub)
        else:
            rospy.logerr("Could not create publishers. Incoming NMEA strings were in wrong format (sentencess must begin with \"$GP\").")
            rospy.logwarn("Node shutting down.")
            rospy.signal_shutdown("Incorrect NMEA input.")
    return publishers


def nmea_collector(ip , port):
    client = setup_connection(ip , port)
    client.settimeout(0)

    #main_antenna_GGA_pub = rospy.Publisher('nmea/main_GGA', Sentence, queue_size=10)
    #secondary_antenna_GGA_pub = rospy.Publisher('nmea/secondary_GGA', Sentence, queue_size=10)
    #HRP_pub = rospy.Publisher('nmea/HRP', Sentence, queue_size=10)

    #main_GGA_sentence = Sentence()
    #HRP_sentence = Sentence()
    #secondary_GGA_sentence = Sentence()

    rate = rospy.Rate(100)
    rospy.loginfo("Starting data retrieval...")

    GPS_data = []
    publishers = []

    while not rospy.is_shutdown():
        try:
            GPS_data = list(client.recv(4096).split('\r\n'))
        except:
            pass

        if(len(publishers) == 0):
            try:
                publishers = create_publishers(GPS_data)
            except:
                pass

        for i in range(0,len(publishers)):
            nmea_sentence = Sentence()
            nmea_sentence.sentence = GPS_data[i]
            publishers[i].publish(nmea_sentence)

        #main_GGA_sentence.sentence = GPS_data[0] if len(GPS_data) >= 1  else ""
        #HRP_sentence.sentence = GPS_data[1] if len(GPS_data) >= 2 else ""
        #secondary_GGA_sentence.sentence = GPS_data[2] if len(GPS_data) >= 3 else ""

        #main_antenna_GGA_pub.publish(main_GGA_sentence)
        #HRP_pub.publish(HRP_sentence)
        #secondary_antenna_GGA_pub.publish(secondary_GGA_sentence)

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
