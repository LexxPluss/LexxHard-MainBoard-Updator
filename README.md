# Main board firmware updator
If you build the image in the local environment, there is no need to do `docker save` and `docker load`.
## docker image Build
```shell
$ docker compose -f release/docker-compose.yml build lexxhard_mainboard_updator
$ docker save -o release/lexxhard_mainboard_updator.tar lexxhard_mainboard_updator

$ cp <confirmed_firmware.bin> release/firmware/

```
## Update firmware
```shell
$ cd release
$ docker load -i lexxhard_mainboard_updator.tar
$ docker compose run --rm lexxhard_mainboard_updator <confirmed_firmware.bin>
```
## Manual Build
```shell
$ source /opt/ros/melodic/setup.bash
$ catkin_make
```
## Manual Update firmware
```shell
$ source devel/setup.bash
$ rosrun mainboard_updator mainboard_updator <confirmed_firmware.bin> [reboot]
```
## Update with legacy ROS system
Please use [legacy](https://github.com/LexxPluss/LexxHard-MainBoard-Updator/tree/legacy) branch to update firmware with legacy ROS system.
