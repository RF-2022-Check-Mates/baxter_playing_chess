#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates

# Get names of the pieces spawned by the spawn_chessboard
piece_names = rospy.get_param('piece_names')

# Global variable where the object's pose is stored
pieces_poses = {}

# Update the state of pieces_poses containing the poses of the pieces
def get_links_gazebo(link_states_msg):
    # Call back to retrieve the object you are interested in
    global piece_names
    global pieces_poses

    poses = {'world': link_states_msg.pose[0]} # get world link
    for (link_idx, link_name) in enumerate(link_states_msg.name):
        modelname = link_name.split('::')[0]

        # Update poses of all pieces in piece_names
        if modelname in piece_names:
            poses[modelname] = link_states_msg.pose[link_idx]
            pieces_poses[modelname] = poses[modelname]




def main():
    rospy.init_node('gazebo2tfframe')

    # Create TF broadcaster -- this will publish a frame give a pose
    tfBroadcaster = tf.TransformBroadcaster()
    # Subscribe to Gazebo's topic where all links and objects poses within the simulation are published
    rospy.Subscriber('gazebo/link_states', LinkStates, get_links_gazebo)

    rospy.loginfo('Spinning')
    global piece_names
    global pieces_poses
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # Print piece_names to see if the service loaded them correctly
        print(piece_names)

        # Continue if there are no poses in the dictionary yet
        if pieces_poses:
            for key,value in pieces_poses.items():
                pos = value.position
                ori = value.orientation

                # Update the position of the piece
                rospy.loginfo((key, pos))

                # Publish transformation given in pose
                tfBroadcaster.sendTransform((pos.x, pos.y, pos.z - 0.93), (ori.x, ori.y, ori.z, ori.w), rospy.Time.now(), key, 'world')
                rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main()
