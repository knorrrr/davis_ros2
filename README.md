# ROS 2 driver for an iniVation event camera
iniVation DAVIS240C driver for ROS 2.

# Prerequirements 
Ubuntu 22.04 LTS, ROS 2 Humble

```bash
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt-get update
sudo apt-get install libcaer-dev
```

# Install
```bash
mkdir -p ros2_ws/src && cd ros2_ws/src
sudo git clone git@github.com:knorrrr/davis_ros2.git
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```


# Davis Driver
## Usage
```bash
source install/setup.bash
ros2 run davis_ros2 davis_pub
```
## Topics 
| Topic name        | Description                            
| ---------------   | ---------------------
| /davis/events     | Events                                  
| /davis/image_raw  | Image Frame(mono16)

# Events Viewer
## Usage
```bash
source install/setup.bash
ros2 run event_viewer event_viewer
```
## Parameter 
| Name                   | Default
| --------------------   | ---------------------
| 'input_topic_name'     | /davis/events 
| 'ouput_topic_name'     | /davis/image_raw 

# Q&A
Q. To increase the hz of /davis/events

A. Declease max_container_interval in Dynamic Reconfigure.

(If this parameter is lowered too much, the frame(/davis/image_waw) becomes unstable.) 