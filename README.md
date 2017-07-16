# ros_braccio_moveit
ROS MoveIt! configuration files for Arduino.ORG's Braccio Arm. This package has been tested on a Braccio Arm Shield v4 connected to a Teensy 3.1. The binary STL files and URDF can be found [here](https://github.com/grassjelly/ros_braccio_urdf)

![braccio_rviz](https://github.com/grassjelly/ros_braccio_moveit/blob/master/img/Screenshot%20from%202017-07-14%2020:02:12.png?raw=true)

# Running the demo
## 1. Download the packages

    cd ~/catkin_ws/src
    git clone https://github.com/grassjelly/ros_braccio_urdf.git
    git clone https://github.com/grassjelly/ros_braccio_moveit.git
    sudo apt-get install ros-indigo-moveit
    cd ..
    catkin_make

## 2. Install PlatformIO:
    sudo apt-get install python-setuptools 
    sudo easy_install pip
    sudo pip install -U platformio
    sudo rm -rf ~/.platformio/

## 3. Define Braccio's serial port on PlatformIO's .ini file:

- Go to the firmware and edit the ini file:

      roscd ros_braccio_moveit/teensy
      nano platformio.ini

- Define Braccio's serial port:
    
      upload_port = /dev/ttyACM0

## 4. Upload the codes:

      platformio run --target upload

    
## 5. Define Braccio's serial port on the launch file:

- Go to launch folder and edit demo.launch:

      roscd ros_braccio_moveit/launch
      nano demo.launch

- Define Braccio's serial port:

      <param name="port" value="/dev/ttyACM0" />

## 6. Run the demo: 

     roslaunch ros_braccio_moveit demo.launch

## 7. Planning and execution
- Click and drag the arrow markers towards your target pose. 
- Go to "Planning" tab and click "Plan and Execute". 

![braccio_planning](https://github.com/grassjelly/ros_braccio_moveit/blob/master/img/Screenshot%20from%202017-07-14%2020:19:56.png?raw=true)
