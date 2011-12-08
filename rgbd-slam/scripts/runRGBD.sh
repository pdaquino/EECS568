#!/bin/bash

# Run RGB-D SLAM on this computer with gamepad input

PROC_CONFIG=$RGBD/config/proc.config

java april.procman.ProcMan -g -d -c $PROC_CONFIG
