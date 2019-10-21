# rembrainbridge_ros

The package is for streaming RGBD images to [Rembrain](www.rembrain.ie) servers for remote robot control.

## Installation

### Step 1

Install ROS kinetic following [ROS instructions](https://wiki.ros.org/kinetic/Installation/Ubuntu)

### Step 2

Create catkin_ws workspace and make it:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

Then clone this repository into carkin_ws/src and apply devel/setup.bash:
```
cd ~/catkin_ws/src/
git clone https://github.com/VasilyMorzhakov/rembrainbridge_ros.git
cd ~/catkin_ws/
source devel/setup.bash
```

### Step 3 

Dependencies:

Install python packages needed for running python scripts:
```
cd ~/catkin_ws/src/rembrainbridge_ros/
sudo -H pip install -r requirements.txt
```

Install ROS dependencies:
```
sudo apt-get install ros-kinetic-cv-bridge
sudo apt-get install ros-kinetic-opencv3 
```

(TODO: check if other dependencies are required)

## Launch

```
cd ~/catckin_ws/src/rembrainbridge_ros/launch
cp main_default.launch main.launch
```
And change launch parameters described below in main.launch.

Then you can run the streamer by roslaunch:

```
roslaunch rembrainbridge_ros main.launch
```

### Launch parameters

- **SERVER_ADDRESS** - remote server name to connect
- **LOGIN** - login robots uses to connect
- **PASSWORD** - password
- **ROBOT_ID** - unique robot identificator
- **width** - RGB image width, which should be the same for depth images
- **height** - height
- **rgb_topic** - name of the topic with RGB images
- **depth_ropic** - name of the topic with depth images

## License

MIT License
