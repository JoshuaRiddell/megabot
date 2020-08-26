#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

source ~/.rosenv
cd $DIR/../ros/catkin_ws
catkin build $1
