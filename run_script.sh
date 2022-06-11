################# IMPORTANT !!! #####################################
# (Terminal 1) roslaunch (ros-gazebo simulator)
roslaunch gazebo_drone iris_world.launch


# (Terminal 2) run firmware (based on mavproxy)
cd ~/AL_SITL/apm/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris


# (Terminal 3) Run gazebo_drone with heli detecting model
conda activate {ENV_NAME}
rosrun gazebo_drone landing_test.py -m {MODEL_PATH}


# (Terminal 4) check image from topic list
rqt

