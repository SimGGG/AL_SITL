################# IMPORTANT !!! #####################################
# (Terminal 1) roslaunch (ros-gazebo simulator)
roslaunch gazebo_ros iris_world.launch


# (Terminal 2) run firmware (based on mavproxy)
cd ~/courseRoot/apm/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris


# (Terminal 3) Run gazebo_drone with heli detecting model
conda activate heli-py37-tf2
rosrun gazebo_drone gazebo_precision_landing4.py -m /home/user/Helipad_Detection/workspace/exported-models/{model}


# (Terminal 4) check image from topic list
rqt

