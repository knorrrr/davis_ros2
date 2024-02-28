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
git clone https://github.com/knorrrr/davis_ros2.git
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```


# Davis Driver
## Usage
```bash
source install/setup.bash
ros2 launch davis_ros2 driver.launch.xml
```
## Topics 
### Output (Default)
| Name                                | Type                                            | Description                           |
| ----------------------------------- | ----------------------------------------------- | ------------------------------------- |
| `~/output/event`                        | `ev_msgs::msg::EventArray`                 | Event information acquired by the event camera.　　|
| `~/output/image`                         | `sensor_msgs::msg::Image`                  | Camera information acquired by the event camera. |

# Events Viewer
## Usage
```bash
source install/setup.bash
ros2 launch event_viewer event_viewer.launch.xml
```
## Topics
### Input (Default)
| Name                                | Type                                            | Description                           |
| ----------------------------------- | ----------------------------------------------- | ------------------------------------- |
| `~/input/event`                     |  `ev_msgs::msg::EventArray`                     | Event information to be converted into images.|

### Output (Default)
| Name                                | Type                                            | Description                           |
| ----------------------------------- | ----------------------------------------------- | ------------------------------------- |
| `~/output/image`                      | `sensor_msgs::msg::Image`                       | Image with projected event information.                          |

# Q&A
Q. To increase the hz of /davis/events

A. Declease max_container_interval in Dynamic Reconfigure.

(If this parameter is lowered too much, the frame(/davis/image_waw) becomes unstable.) 