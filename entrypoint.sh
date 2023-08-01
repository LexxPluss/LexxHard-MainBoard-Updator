#!/bin/bash

source ./devel/setup.bash
check_main_board=$(rosnode info main_board_topic_node 2>&1 | grep "ERROR: Unable to communicate with master")
if [ -n "$check_main_board" ]; then
  echo "ERROR: Unable to communicate with main_board."
  echo "Please run LexxAuto or hybrid-amr-release"
  exit
else
  echo "Enable to communicate with main_board."
fi
echo $check_main_board
rosrun mainboard_updator mainboard_updator /LexxHard-MainBoard-Updator/firmware/$@
echo "Please reboot."