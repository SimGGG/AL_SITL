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
	cd ~/AL_STIL
	mkdir apm && cd apm
	git clone -b Copter-4.1.1 https://github.com/ardupilot/ardupilot && cd ardupilot
	git submodule update --init --recursive
	```

- Step 2. Add Ardupilot firmware path
	```
	echo 'export PATH=$PATH:/home/$USER/AL_SITL/apm/ardupilot/Tools/autotest' >> ~/.bashrc
	```

- Step 3. (Optional) Check available board list
	```
	cd ~/AL_SITL/apm/ardupilot
	./waf list_boards
	```


## :white_check_mark: ardupilot_gazebo

I assume you already have Gazebo installed with ROS (or without).
If you don't have it yet, install ROS with `sudo apt install ros-melodic-desktop-full`
(follow instruction here http://wiki.ros.org/melodic/Installation/Ubuntu).
Due to a bug in current gazebo release from ROS, please update gazebo with OSRF version from http://gazebosim.org/tutorials?tut=install_ubuntu

- Step 1. git clone ardupilot_gazebo related on Gazebo World & Models
	```
	cd ~/AL_SITL/ardupilot_gazebo
	mkdir build && cd build
	cmake ..
	make -j4
	sudo make install
	```

- Step 2. Add path related on gazebo source
	```
	echo 'source /usr/share/gazebo-9/setup.sh' >> ~/.bashrc
	echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/$USER/AL_SITL/ardupilot_gazebo/models' >> ~/.bashrc
	echo 'export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/$USER/AL_SITL/ardupilot_gazebo:/home/$USER/AL_SITL/ardupilot_gazebo/worlds' >> ~/.bashrc
	```


## :white_check_mark: ROS workspace setup

- Step 1. create ROS Workspace
	```
	mkdir -p ~/AL_SITL/catkin_ws/src
	```

- Step 2. Install `ros_numpy` in ROS workspace
	```
	cd catkin_ws/src
	git clone https://github.com/eric-wieser/ros_numpy
	```

- Step 3. Create project directory for ROS - `gazebo_drone` (This will be our ROS project directory)
	```
	catkin_create_pkg gazebo_drone rospy roscpp
	```

- Step 4. Build the ROS workspace
	```
	cd ..
	catkin_make
	```

- Step 5. Add ROS source path. It makes our ROS workspace overlay on ROS melodic base
	```
	echo 'source /home/$USER/AL_SITL/catkin_ws/devel/setup.bash' >> ~/.bashrc
	```

- Step 6. Move `scripts` and `launch` to ROS project directoy(`~/AL_SITL/catkin_ws/src/gazebo_drone`)
	```
	cd ~/AL_SITL
	mv launch/ catkin_ws/src/gazebo_drone/
	mv scripts/ catkin_ws/src/gazebo_drone/
	```
	



## :warning: Tensorflow Object Detection API Installation

- Step 1. Create a new Ananconda virtual environment
	```
	conda create -n <ENV_NAME> pip python=3.7
	```

- Step 2. Activate conda environment
	```
	conda activate <ENV_NAME>
	```

- Step 3. Tensorflow Installtion
	```
	pip install --ignore-installed --upgrade tensorflow==2.5.0
	```

- Step 4. Downloading the Tensorflow Model Garden
	```
	git clone https://github.com/tensorflow/models.git ~/Tensorflow/models
	```

- Step 5. Protobuf Installation/Compilation
	```
	protoc object_detection/protos/*.proto --python_out=.
	```

- Step 6. COCO API Installation
	```
	(In a new terminal)
	git clone https://github.com/cocodataset/cocoapi.git
	cd cocoapi/PythonAPI
	make
	cp -r pycocotools ~/Tensorflow/models/research/
	```

- Step 7. Install the Object Detection API
	```
	cp ~/Tensorflow/models/research/object_detection/tf2/setup.py .
	python -m pip install --use-feature=2020-resolver .
	```

## :white_check_mark: Run Gazebo_drone with python scripts

- Step 1. Run python script with ros package `gazebo_drone`. Required Anaconda environment that installed Object Detection API
	Object detection model must be exported by tensorflow pipeline
	```
	conda activate <ENV_NAME>
	```

- Step 2. Run ros node(*.py)
	```
	rosrun gazebo_drone landing_test.py -m <MODEL_PATH>
	```

