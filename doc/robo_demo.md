# Running robo_demo code in simulation and on the real panda
###### David Yackzan

First ensure that everything is installed as outlined in the installation documentation and source your MoveIt workspace. Ex: `source ~/robo_demo/devel/setup.zsh`

### Simulation
1. Start up rviz to see the Panda simulation. Type the following in a terminal: \
`roslaunch panda_moveit_config demo.launch`
###### Command line interface
2. Start up command line interface for end effector point following. Type the following in a separate terminal: \
`rosrun end_effector_control follow_point.py`
3. Follow the prompts and experiment with different end effector positions
###### Subscriber interface
2. Start up subscriber interface for end effector point following. Type the following in a separate terminal: \
`rosrun end_effector_control follow_point_subscriber.py`
3. Publish messages to the `/point_command` topic to control the Panda end effector position. The method for doing so is outlined in the `~/robo_demo/src/end_effector_control/scripts/cmd_point_publisher`. This program can also be run from a separate terminal by typing in `Python` and then copy pasting the code.

### Real Panda
1. Switch on the panda control box on
2. Type in `172.16.0.2` to your browser to open the panda api (this may take a few minutes to load and connect to the panda while the control box is booting up)
3. Unlock the joints on the panda bot by pressing the "unlock joints" button in the browser api
4. Hold down the safety switch hand trigger before and while running the following programs to enable the panda bot to move
5. Connect rviz to the Panda simulation. Type the following in a terminal: \
`roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=172.16.0.2 load_gripper:=true`
###### Command line interface
6. Start up command line interface for end effector point following. Type the following in a separate terminal: \
`rosrun end_effector_control follow_point.py`
7. Follow the prompts and experiment with different end effector positions
###### Subscriber interface
6. Start up subscriber interface for end effector point following. Type the following in a separate terminal: \
`rosrun end_effector_control follow_point_subscriber.py`
7. Publish messages to the `/point_command` topic to control the Panda end effector position. The method for doing so is outlined in the `~/robo_demo/src/end_effector_control/scripts/cmd_point_publisher`. This program can also be run from a separate terminal by typing in `Python` and then copy pasting the code.


####<span style="color:aqua">Troubleshooting</span>
<span style="color:red">Error</span>: Failed to load position_joint_trajectory_controller \
<span style="color:green">Solution</span>: `sudo apt-get install ros-melodic-joint-trajectory-controller`

<span style="color:red">Error</span>: Not able to move in current configuration \
<span style="color:green">Solution</span>: ensure that panda controller switch is turned on, the joints are unlocked (via the api to be opened in your browser), and the hand trigger safety switch is pressed.
