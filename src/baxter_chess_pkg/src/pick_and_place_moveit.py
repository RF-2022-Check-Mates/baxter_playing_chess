#!/usr/bin/env python
#on steroids
import sys
import copy

import rospy
import rospkg

# Importing TF to facilitate the task of receiving transformations
import tf2_ros

from std_msgs.msg import (
    Empty,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

import baxter_interface
import moveit_commander


#
#    Our choreography
#       - First part of each tuple is the name of the piece
#       - Second is the coordinate we want the piece to be moved to
moves = [
    ("R1","53"),
    ("k3","22"),
    ("R6","62"),
    ("k3","21"),
    ("R1","51"),
    ("k3","20"),
    ("R6","60"),
]

class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        self._limb_name = limb  # string
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")

        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        # We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")
        print("Successfully initialized!\n")

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))

        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance

        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        # servo up from current pose
        self._group.set_pose_target(ik_pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def _servo_to_pose(self, pose):
        # servo down to release
        self._group.set_pose_target(pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        rospy.sleep(1.5)
        # servo to pose
        current_pose = copy.deepcopy(pose)
        current_pose.position.z += 0.06
        self._servo_to_pose(current_pose)
        rospy.sleep(1.0)

        # Lower the speed of the movement to avoid jitter
        self._group.set_max_velocity_scaling_factor(0.5)
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

        # Change speed back to normal
        self._group.set_max_velocity_scaling_factor(1.0)


    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        rospy.sleep(1.5)
        current_pose = copy.deepcopy(pose)

        # Offset to drop the piece from
        current_pose.position.z += 0.03

        # Lower the movement speed
        self._group.set_max_velocity_scaling_factor(0.5)
        self._servo_to_pose(current_pose)

        # Wait until the arm stops shaking from the movement
        rospy.sleep(2.2)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

        # Change speed back to normal
        self._group.set_max_velocity_scaling_factor(1.0)

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ik_pick_and_place_moveit")

    # This creates a transform listener object; once created, it starts receiving
    # transformations using the /tf topic and buffers them up for up to 10 seconds.
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15  # meters

    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    # NOTE: Gazebo and Rviz has different origins, even though they are connected. For this
    # we need to compensate for this offset which is 0.93 from the ground in gazebo to
    # the actual 0, 0, 0 in Rviz.
    starting_pose2 = Pose(
        position=Point(x=0.2, y=0.35, z=0.25),
        orientation=overhead_orientation)
    pnp = PickAndPlaceMoveIt(limb, hover_distance)

    starting_pose = Pose(
        position=Point(x=0.7, y=0.135, z=0.35),
        orientation=overhead_orientation)
    pnp = PickAndPlaceMoveIt(limb, hover_distance)


    # Move to the desired starting angles
    pnp.move_to_start(starting_pose)
    idx = 0
    rate = rospy.Rate(1.0)
    piece_names = rospy.get_param('piece_names')
    piece_target_position_map = rospy.get_param("piece_target_position_map")
    print(type(piece_target_position_map))
    print(piece_target_position_map.keys())
    while not rospy.is_shutdown():
        # Perform the choreography
        for move in moves:
            try:
                transformation = tfBuffer.lookup_transform('base', move[0], rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            # Get the position of the piece
            current_pose = Pose(
            position=Point(x=transformation.transform.translation.x, y=transformation.transform.translation.y, z=transformation.transform.translation.z - 0.02),
            orientation=overhead_orientation)
            print("\nPicking...")

            # Pick the piece
            pnp.pick(current_pose)
            print("\nPlacing...")

            # Get the position from the piece_target_position_map provided by spawn_chessboard
            x,y,z = piece_target_position_map[move[1]]

            # Drop the piece to the wanted position
            pnp.place(Pose(
                position=Point(x=x, y=y, z=z),
                orientation=overhead_orientation))

        return 0


if __name__ == '__main__':
    sys.exit(main())
