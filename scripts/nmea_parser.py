#!/usr/bin/env python

import socket
import rospy
import sys
from nmea_msgs.msg import Sentence


def create_publishers(GPS_data):
    publishers = []
    nmea_sentences = []
    for i in range(0,len(GPS_data)):
        if(str(GPS_data[i]).startswith("$")):
           nmea_sentences.append(GPS_data[i])

    for i in range(0,len(nmea_sentences)):
        if(str(nmea_sentences[i]).startswith("$")):
            pub = rospy.Publisher(
                                "nmea/" + str(nmea_sentences[i]).split(',')[0][3:len(str(nmea_sentences[i]).split(',')[0])],
                                Sentence,
                                queue_size=10
                                )
            publishers.append(pub)
        else:
            rospy.logerr("Could not create publishers. Incoming NMEA strings were in wrong format (sentences must begin with \"$GP\").")
            rospy.logwarn("Node shutting down.")
            rospy.signal_shutdown("Incorrect NMEA input.")
    return publishers


def nmea_collector(ip, port):
    client = setup_connection(ip , port)
    client.settimeout(0)

    rate = rospy.Rate(100)
    rospy.loginfo("Starting data retrieval...")

    GPS_data = []
    publishers = []

    while not rospy.is_shutdown():
        try:
            GPS_data = list(client.recv(1024).split('\r\n'))
        except socket.error as e:
            if not e.errno == 11: #errorno 11 -> Resource temporarily unavailable
                rospy.logerr(str(e))
            pass

        if(len(publishers) == 0):
            try:
                publishers = create_publishers(GPS_data)
            except Exception as e:
                rospy.logerr(str(e))
                pass

        try:
            for i in range(0,len(publishers)):
                nmea_sentence = Sentence()
                time = rospy.get_rostime()
                nmea_sentence.header.stamp.secs = time.secs
                nmea_sentence.header.stamp.nsecs = time.nsecs
                nmea_sentence.sentence = GPS_data[i]
                publishers[i].publish(nmea_sentence)
        except Exception as e:
            if (type(e) == type(IndexError())):
                rospy.logerr("Stopped receiving data. \nError: %s", e)
                rospy.logwarn("Node shutting down.")
            else:
                rospy.logerr(str(e))
            rospy.signal_shutdown("")

        rate.sleep()

    rospy.spin()


def setup_connection(_ip, _port):
    port = _port
    ip = None
    attempts_limit = 10
    current_attempt = 0
    connected = False

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(5)
    ip = socket.gethostbyname(_ip)
    address = (ip, port)

    while not connected and not rospy.is_shutdown() and current_attempt < attempts_limit:
        current_attempt += 1

        try:
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
        rospy.logerr("No connection established.")
        rospy.logwarn("Node shutting down.")
        sys.exit()

    return client


if __name__ == '__main__':
    try:
        rospy.init_node('NMEA_parser')
        if not(2 < len(sys.argv) <= 3) or not str(sys.argv[1]).__contains__("ip:") or not str(sys.argv[2]).__contains__("port:"):
            rospy.logwarn("Incorrect passing of arguments. Please try again using the following format: \n"
                          "rosrun nmea_parser nmea_parser.py ip:xxx.xxx.xxx.xxx port:xxxxx")
        else:
            nmea_collector(str(sys.argv[1]).split(":")[1], int(str(sys.argv[2]).split(":")[1]))
    except rospy.ROSInterruptException:
        pass
