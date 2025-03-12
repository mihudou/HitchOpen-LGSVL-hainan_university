#!/bin/bash

ROS_DISTRO_SOURCE=/opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${ROS_DISTRO_SOURCE}" ]; then
    source ${ROS_DISTRO_SOURCE}
fi

LOCAL=install/setup.bash;
if [ -f "${LOCAL}" ]; then
    source ${LOCAL};
fi;