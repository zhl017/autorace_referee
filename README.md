## AutoRace Referee
![](https://github.com/zhl017/omiyage/blob/main/Documents/images/2022_autorace_referee.png)

## How to use
- put each in the folder `ros_package` into your ROS catkin workspace, and `catkin_make`:
```
~/catkin_ws/src/
```
## Connect OpenCR to PC via USB connector
- /dev/ttyACM0  >>> Traffic Light

- /dev/ttyACM1  >>> Challenge Stage

- /dev/ttyACM2  >>> Level Crossing

## Launch the referee_monitor
```
roslaunch rbiz_autorace_monitor rbiz_autorace_monitor.launch
```

## If you need more information, please contact `info@idminer.com.tw`
