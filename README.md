# AL_SITL(Autonomous Landing for SITL)
## Prerequisite
- Ubuntu 18.04 LTS
    - Gazebo 9
    - ROS Melodic
    - gazebo_ros_pkgs
    - Anaconda
    

## :white_check_mark: Build Ardupilot

- Step 1. git clone Ardupilot
	```
	$ cd ~/AL_STIL
	$ mkdir apm && cd apm
	$ git clone -b Copter-4.1.1 https://github.com/ardupilot/ardupilot && cd ardupilot
	$ git submodule update --init --recursive
	```

- Step 2. Add Ardupilot firmware path
	```
	$ echo 'export PATH=/home/{USER_NAME}/AL_SITL/apm/ardupilot/Tools/autotest' >> ~/.bashrc
	```

- Step 3. (Optional) Check available board list
	```
	$ cd ~/AL_SITL/apm/ardupilot
	$ ./waf list_boards
	```


## :white_check_mark: ardupilot_gazebo
I assume you already have Gazebo installed with ROS (or without).  
If you don't have it yet, install ROS with `sudo apt install ros-melodic-desktop-full`
(follow instruction here http://wiki.ros.org/melodic/Installation/Ubuntu).  
Due to a bug in current gazebo release from ROS, please update gazebo with OSRF version from http://gazebosim.org/tutorials?tut=install_ubuntu

- Step 1. git clone ardupilot_gazebo related on Gazebo World & Models
	````
	$ cd ~/AL_SITL/ardupilot_gazebo
	$ mkdir build && cd build
	$ cmake ..
	$ make -j4
	$ sudo make install
	````

- Step 2. Add path related on gazebo source
	````
	$ echo 'source /usr/share/gazebo-9/setup.sh' >> ~/.bashrc
	$ echo 'export GAZEBO_MODEL_PATH=/home/{USER_NAME}/AL_SITL/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc
	$ echo 'export GAZEBO_RESOURCE_PATH=/home/{USER_NAME}/AL_SITL/ardupilot_gazebo/worlds:$GAZEBO_RESOURCE_PATH' >> ~/.bashrc
	````


## :white_check_mark: ROS workspace setup

- Step 1. Build base ROS Workspace
	````
	$ cd ~/AL_SITL
	$ mkdir -p catkin_ws/src
	$ cd catkin_ws
	$ catkin_make
	````

- Step 2. Add ROS source path
	````
	$ echo "source /home/{USER_NAME}/AL_SITL/catkin_ws/devel/setup.bash" >> ~/.bashrc
	````

- Step 3. Install `ros_numpy`
	```
	$ cd ../src
	$ git clone https://github.com/eric-wieser/ros_numpy
	```

- Step 4. Create project directory with ros package - `gazebo_drone` (This will be our ros project directory)
	```
	$ catkin_create_pkg gazebo_drone rospy roscpp
	```

- Step 5. Build project ROS workspace
	```
	$ cd ..
	$ catkin_make
	```



## :white_check_mark: Run Gazebo_drone with python scripts

- Step 1. Make python script workspace
	```
	$ cd ~/AL_SITL/catkin_ws/src/gazebo_drone
	$ mkdir scripts
	```

- Step 2. Copy all python scripts in `landing_scripts` to `catkin_ws/src/gazebo_drone/scripts`
	```
	$ cd ~/AL_SITL
	$ cp landing_scripts/* catkin_ws/src/gazebo_drone/scripts
	```

- Step 3. Create Conda environment

	Since `landing_test.py` contains Obeject Detection module, need to be installed tensorflow and other python packages. We use Anaconda virtual environment and `environment.yaml` is our conda environment files.
	Install manual as below..
	```
	$ conda env create --file ~/AL_SITL/environment.yaml
	```

- Step 4. Run python script with ros package `gazebo_drone`

	Object detection model must be exported by tensorflow pipeline
	```
	$ conda activate heliipy37-tf2
	$ rosrun gazebo_drone landing_test.py -m ~/AL_SITL/object_detection/{MODEL}
	```

## :construction: 


