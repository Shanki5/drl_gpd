FROM andrejorsula/drl_grasping:latest

# Create a colcon workspace
RUN source /opt/ros/rolling/setup.bash \
 && mkdir -p fyp_ws/src

### Add entrypoint sourcing the environment
COPY ./entrypoint.bash ./entrypoint.bash

# Communicate within localhost only
ENV ROS_LOCALHOST_ONLY=1
# Set domain ID for ROS2 in order to not interfere with host
ENV ROS_DOMAIN_ID=69

### Set entrypoint and default command
ENTRYPOINT ["/bin/bash", "-c", "source ${DRL_GRASPING_DIR}/entrypoint.bash && \"$@\"", "-s"]
CMD ["/bin/bash"]
