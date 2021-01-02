#!/bin/bash

service avahi-daemon restart

source /opt/ros/kinetic/setup.bash
exec "$@"
