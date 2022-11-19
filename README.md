# abot04_pkgs
Abot04 ROS packages

## Environment
- OS : Ubuntu 20.04 LTS
- ROS : noetic
- ROS2 : Not supported
- Python: 3.8.0
- Odrive : python3 odrive == 0.6.3.post0


## Installation
### Odrive Setup
```
sudo apt install python3 python3-pip
pip3 install matplotlib
pip3 install --upgrade odrive # or pip3 install odrive==0.6.3.post0
sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"
```
Then restart you PC.

```bash
mkdir -p <catkin_ws>/src
cd <catkin_ws>/src
vcs import < abot04_pkgs/dependencies.repos
cd <catkin_ws>
rosdep install -i -y -r --from-paths src
catkin b
source devel/setup.bash
```


## Execution
You need at leas 2 terminals.

### Terminal 1
Run ROS core.
```bash
roscore
```

### Terminal 2
With following command, you can control the robot with Twist message.
```bash
cd <catkin_ws>
source devel/setup.bash
roscd abot04_driver/script
./twist_driver_run.bash
```

If you need to control via Joy Stick, the following command will work on new launched terminal.
```bash
cd <catkin_ws>
source devel/setup.bash
roslaunch joy_twist teleop_twist_joy.launch
```
