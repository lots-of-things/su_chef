# ros_braccio_moveit
ROS MoveIt! configuration files for Arduino.ORG's Braccio Arm. This package has been tested on a Braccio Arm Shield v4 connected to a Teensy 3.1. The binary STL files and URDF can be found [here](https://github.com/grassjelly/ros_braccio_urdf)

![braccio_rviz](https://github.com/grassjelly/ros_braccio_moveit/blob/master/img/Screenshot%20from%202017-07-14%2020:02:12.png?raw=true)

# Running the demo
## 1. Install MoveIt:

    sudo apt-get install ros-indigo-moveit

## 2. Define Braccio's serial port:
Edit [demo.launch](https://github.com/grassjelly/ros_braccio_moveit/blob/master/launch/demo.launch#L50) and define your Braccio Arm's serial port:

    <param name="port" value="/dev/ttyACM0" />

## 3. Run the demo: 

     roslaunch ros_braccio_moveit demo.launch

## 4. Planning and execution
- Click and drag the arrow markers towards your target pose. 
- Go to "Planning" tab and click "Plan and Execute". 

![braccio_planning](https://github.com/grassjelly/ros_braccio_moveit/blob/master/img/Screenshot%20from%202017-07-14%2020:19:56.png?raw=true)
