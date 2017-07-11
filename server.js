const express = require('express');
const exec = require('child_process').exec;
const app = express();

app.use(express.static(__dirname)); // set the static files location 

/* RESTful API for executing bash commands */
app.post('/api/killros', function (req, res) {
  // let child = exec('pwd', function (error, stdout, stderr) {
  function puts(error, stdout) { console.log(stdout); }
  exec('pwd', puts);
})


let server = app.listen(8888, '0.0.0.0', function () {

  const host = server.address().address;
  const port = server.address().port;

  console.log("Server listening at http://%s:%s", host, port);

})
