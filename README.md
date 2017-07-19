# Web Control Center

based on the ROS Control Center: https://github.com/pantor/ros-control-center

![screenshot](https://github.com/AutoModelCar/web-control-center/raw/master/assets/images/screenshot.png)

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

Or to run everything together:

```sh
    $ ./start-server.sh
```

## Open the Web Control Center

The control center runs on any computer (in particular without ROS...) in the same network. Open the Web Control Center at the URL of the server (for example: `http://192.168.1.178:8888`). In the settings tab, you need to enter the IP address and port (9090 by default) of your robot. In the *Image Preview Port* you need to enter the port of the `web_video_server`, which should be 8080 by default. Open the `Control` tab and reload.

> When the server is not the same machine as the robot, then make sure the same ROS message types are installed on both machines.

## Compiling

When you are doing changes on the code you have to compile the project with `gulp`. A linter checks the code for errors first and then all javascript files are merged into a few big ones. To do everything in one step you can easily call:

```sh
    $ cd web-control-center
    $ npm run compile
```
