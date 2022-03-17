#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy

# http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials

if __name__ == '__main__':
    rospy.init_node("spawn_chessboard")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    srv_call = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    # Table
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    table_xml = ''
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace('\n', '')

    move_in_x = 0
    move_in_y = 0
    table_pose=Pose(position=Point(x=0.73+move_in_x, y=0.4 + move_in_y, z=0.0))
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_sdf("cafe_table", table_xml, "/", table_pose, "world")
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


    # ChessBoard
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    board_pose = Pose(Point(0.3 + move_in_x,0.55 + move_in_y,0.78), orient)
    frame_dist = 0.025
    #model_path = rospkg.RosPack().get_path('chess_baxter')+"/models/"
    model_path = rospkg.RosPack().get_path('baxter_chess_pkg')+"/models/"
    print(model_path)

    with open(model_path + "chessboard/model.sdf", "r") as f:
        board_xml = f.read().replace('\n', '')

    # Add chessboard into the simulation
    print srv_call("chessboard", board_xml, "", board_pose, "world")

    # Add chesspieces into the simulation
    origin_piece = 0.03125

    pieces_xml = dict()
    list_pieces = 'rnbqkpRNBQKP'
    for each in list_pieces:
        with open(model_path + each+".sdf", "r") as f:
            pieces_xml[each] = f.read().replace('\n', '')

    # All the * are necessary as we need the coordinates in the piece_positionmap
    board_setup = [ '********', #0
                    '**r**r**', #1
                    '***k****', #2
                    '********', #3
                    '****K***', #4
                    '*R******', #5
                    '******R*', #6
                    '********'] #7

    # A map of tile coordinates to actual positions
    piece_positionmap = dict()

    # Names of spawned pieces
    piece_names = []
    for row, each in enumerate(board_setup):
        for col, piece in enumerate(each):
            pose = deepcopy(board_pose)

            # Set the piece position
            pose.position.x = board_pose.position.x + frame_dist + origin_piece + row * (2 * origin_piece)
            pose.position.y = board_pose.position.y - 0.55+ frame_dist + origin_piece + col * (2 * origin_piece)
            pose.position.z += 0.018

            # Set the tile coordinate to correspond with an actual position
            piece_positionmap[str(row)+str(col)] = [pose.position.x, pose.position.y, pose.position.z-0.93] #0.93 to compensate Gazebo RViz origin difference

            # Spawn a piece
            if piece in list_pieces:
                piece_names.append("%s%d" % (piece,col))
                print srv_call("%s%d" % (piece,col), pieces_xml[piece], "", pose, "world")


    rospy.set_param('board_setup', board_setup) # Board setup
    rospy.set_param('list_pieces', list_pieces) # List of unique pieces
    rospy.set_param('piece_target_position_map', piece_positionmap) # 3D positions for each square in the chessboard
    rospy.set_param('piece_names', piece_names) # Pieces that will be part of the game
    rospy.set_param('pieces_xml', pieces_xml) # File paths to Gazebo models, i.e. SDF files
