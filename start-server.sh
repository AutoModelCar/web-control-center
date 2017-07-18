#!/bin/bash

roslaunch rosbridge_server rosbridge_websocket.launch &
rosrun web_video_server web_video_server &

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR
npm start
