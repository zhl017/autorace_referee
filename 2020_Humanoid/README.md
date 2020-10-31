# 2020_Humanoid
![](https://i.imgur.com/cKqtu9q.jpg)

## How to use
- upload each source in the folder `OpenCR_Code` into each OpenCR

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
