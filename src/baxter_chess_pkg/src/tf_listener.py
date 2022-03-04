#!/usr/bin/env python  
import rospy
import math

# Importing TF to facilitate the task of receiving transformations
import tf2_ros

if __name__ == '__main__':
    rospy.init_node('baxter_tf_listener')

    # This creates a transform listener object; once created, it starts receiving
    # transformations using the /tf topic and buffers them up for up to 10 seconds.
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # This is where the magic happens, a query is passed to the listener for the
    # /base to /block transform by means of the lookupTransform fn. The arguments are
    # from "this frame" to "this frame" at "this specific time"
    # (if you pass "rospy.Time(0), the fn will give you the latest available transform 
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            list_pieces = rospy.get_param('list_pieces')
            board_setup = rospy.get_param('board_setup')
            for row, each in enumerate(board_setup):
                for col, piece in enumerate(each):
                    if piece in list_pieces:
                        piece_name = "%s%d" % (piece, col)
                        transformation = tfBuffer.lookup_transform('base', piece_name, rospy.Time())
                        rospy.loginfo("Translation: \n" + str(transformation.transform.translation))
                        rospy.loginfo("Quaternion: \n" + str(transformation.transform.rotation))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        

        rate.sleep()