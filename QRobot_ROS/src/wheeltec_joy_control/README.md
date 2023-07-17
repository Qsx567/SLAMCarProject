# 轮趣的PS2遥控器使用方法

## 1、查看输入
```
ls /dev/input
```

## 2、测试
```
sudo apt-get install joystick
sudo jstest /dev/input/js0
```

## 3、编译功能包
```
catkin_make -DCATKIN_WHITELIST_PACKAGES="wheeltec_joy_control"
roslaunch wheeltec_joy joy_data.launch
```

