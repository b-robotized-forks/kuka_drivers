1. Load the hardware

_Currently deployed version with GPIOs_

Short (possible because of default values):
```
ros2 launch kuka_rsi_driver description.launch.xml
```

Full:
```
ros2 launch kuka_rsi_driver description.launch.xml robot_familiy:=agilus robot_model:=kr10_r900_2 controller_ip:=10.28.23.240 driver_version:=eki_rsi client_ip:=10.23.23.28 client_port:=28283 verify_robot_model:=false use_gpio:=true
```


2. Load controllers and the robot manager component:
```

```
