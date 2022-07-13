# Main board firmware updator

## Build
```shell
$ source /opt/ros/melodic/setup.bash
$ catkin_make
```

## Update firmware
```shell
$ source devel/setup.bash
$ rosrun mainboard_updator mainboard_updator <confirmed.bin> [reboot]
```
