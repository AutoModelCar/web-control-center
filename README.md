# Web Control Center

based on the ROS Control Center: https://github.com/pantor/ros-control-center

![screenshot](https://raw.githubusercontent.com/pantor/ros-control-center/master/assets/images/screenshot.png)

## Installation

1. Install RosbridgeSuite package and WebVideoServer

    ```sh
    $ sudo apt-get install ros-kinetic-rosbridge-suite
    $ sudo apt-get install ros-kinetic-web-video-server
    ```
2. Install NodeJS with NPM  
  
    https://nodejs.org/en/download/

3. Install NodeJS dependencies for web-control-center

    ```sh
    $ cd web-control-center
    $ npm install
    ```
    
## Run the server

```sh
    $ roslaunch rosbridge_server rosbridge_websocket.launch &
    $ rosrun web_video_server web_video_server &
    $ cd web-control-center
    $ npm start
```

## Open the Web Control Center

The control center runs on any computer (in particular without ROS...) in the same network. Open the Web Control Center at the URL of the server (for example: `http://192.168.1.178:8888`). In the settings tab, you need to enter the IP address and port (9090 by default) of your robot. In the *Image Preview Port* you need to enter the port of the `web_video_server`, which should be 8080 by default. Open the `Control` tab and reload.

