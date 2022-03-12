# Baxter playing chess
This is our implementation of robot Baxter playing chess. The game presented has 7 hardcoded moves, delivering checkmate on the white king.

## How to run the simulation
* ***In terminal 1 run:*** roslaunch baxter_gazebo baxter_world.launch
* ***In terminal 2 run:*** rosrun baxter_tools enable_robot.py -e && rosrun baxter_interface joint_trajectory_action_server.py
* ***In terminal 3 run:*** roslaunch baxter_moveit_config baxter_grippers.launch
* ***In terminal 4 run:*** cd ~/baxter_playing_chess && catkin_make && source devel/setup.bash
* ***In terminal 4 run:*** rosrun baxter_chess_pkg pick_and_place_moveit.py   -> this will crash, but it is initially used to move hand to starting position before spawning models
* ***In terminal 5 run:*** cd ~/baxter_playing_chess && catkin_make && source devel/setup.bash
* ***In terminal 5 run:*** rosrun baxter_chess_pkg spawn_chessboard.py && rosrun baxter_chess_pkg gazebo2tfframe.py
* ***In terminal 4 run:*** rosrun baxter_chess_pkg pick_and_place_moveit.py   -> this time it should run the simulation till the end

