# jetbot_ros
ROS nodes and Gazebo model for NVIDIA JetBot with Jetson Nano



### Install Adafruit Libraries

These Python libraries from Adafruit support the TB6612/PCA9685 motor drivers and the SSD1306 debug OLED:

```bash
# pip should be installed
$ sudo apt-get install python-pip

# install Adafruit libraries
$ pip install Adafruit-MotorHAT
$ pip install Adafruit-SSD1306
```

Grant your user access to the i2c bus:

```bash
$ sudo usermod -aG i2c $USER
```

Reboot the system for the changes to take effect.


### Build jetson-inference

Clone and build the [`jetson-inference`](https://github.com/dusty-nv/jetson-inference) repo:

```bash
# git and cmake should be installed
sudo apt-get install git cmake

# clone the repo and submodules
cd ~/workspace
git clone https://github.com/dusty-nv/jetson-inference
cd jetson-inference
git submodule update --init

# build from source
mkdir build
cd build
cmake ../
make

# install libraries
sudo make install
```


### Using the Camera

To begin streaming the JetBot camera, start the `jetbot_camera` node:

```bash
$ rosrun jetbot_ros jetbot_camera
```

The video frames will be published to the `/jetbot_camera/raw` topic as [`sensor_msgs::Image`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) messages with BGR8 encoding.  To test the camera feed, install the [`image_view`](http://wiki.ros.org/image_view?distro=melodic) package and then subscribe to `/jetbot_camera/raw` from a new terminal:

```bash
# first open a new terminal
$ sudo apt-get install ros-melodic-image-view
$ rosrun image_view image_view image:=/jetbot_camera/raw
```

A window should then open displaying the live video from the camera.  By default, the window may appear smaller than the video feed.  Click on the terminal or maximize button on the window to enlarge the window to show the entire frame.


## JetBot Model for Gazebo Robotics Simulator

<img src="https://github.com/dusty-nv/jetbot_ros/raw/master/gazebo/jetbot_gazebo_0.png" width="700">

See the [`gazebo`](gazebo) directory of the repo for instructions on loading the JetBot simulator model for Gazebo.

