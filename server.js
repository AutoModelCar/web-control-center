"use strict";

const express = require('express');
const exec = require('child_process').exec;
const app = express();


class RunBash {
    constructor() {
      this.proc = null;
    }

    puts (error, stdout, stderr) {
      console.log('[RunBash] stdout: ' + stdout);
      if(stderr) {
        console.error('[RunBash] stderr: ' + stderr);
      }
      if (error !== null) {
        console.error('[RunBash] exec error: ' + error);
      }
    }
    
    run (command) {
        // this.proc = exec(command, this.puts);
        this.proc = exec(command);
        console.log('[web-constrol-center] run "' + command + '" with pid=' + (this.proc.pid));
    }
}

const runBash = new RunBash();

function stopAll () {
  runBash.run('pkill -f random_movement');
  runBash.run('pkill -f manual_control');
}

/* RESTful API for executing bash commands */
app.post('/api/randommovement', function (req, res) {
  stopAll();
  runBash.run('roslaunch random_movement auto.launch');
});

app.post('/api/manualcontrol', function (req, res) {  
  stopAll();
  runBash.run('roslaunch manual_control manual_odroid.launch');
});

app.post('/api/stop', function (req, res) {  
  stopAll();
});

app.use(express.static(__dirname)); // set the static files location 

let server = app.listen(8888, '0.0.0.0', function () {

  const host = server.address().address;
  const port = server.address().port;

  console.log("Server listening at http://%s:%s", host, port);

})
