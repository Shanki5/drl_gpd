### Source ROS2
source /opt/ros/rolling/setup.bash

### Source ROS2 <-> IGN
source ~/drl_grasping/ros_ign/install/local_setup.bash

## Appending source command to ~/.bashrc enables autocompletion (ENTRYPOINT alone does not support that)
grep -qxF '. "${DRL_GRASPING_DIR}/entrypoint.bash"' ${HOME}/.bashrc || echo '. "${DRL_GRASPING_DIR}/entrypoint.bash"' >>${HOME}/.bashrc
