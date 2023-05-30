#!/bin/bash
set -e

# setup ros2 environment
echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> ~/.bashrc

# start SSH server for Clion. prints get pushed to /dev/null so they don't clog up the terminal. 
/usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_test_clion > /dev/null 2>&1 & 

exec "$@"
