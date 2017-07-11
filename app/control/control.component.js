let ros;
let isConnected = false;

class ControlController {
  constructor($timeout, $interval, $http, Settings, Domains) {
    this.$http = $http;
    this.$timeout = $timeout;
    this.Domains = Domains;

    // hardcoded list of shown topics
    this.DomainsToShow = ['app', 'manual_control', 'model_car', 'odom', 'usb_cam', 'scan'];

    this.isConnected = isConnected;
    this.setting = Settings.get();
    this.maxConsoleEntries = 200;

    // Load ROS connection and keep trying if it fails
    this.newRosConnection();
    $interval(() => {
      this.newRosConnection();
    }, 1000); // [ms]

    this.resetData();
    if (isConnected) {
      this.onConnected();
    }
  }

  // The active domain shows further information in the center view
  setActiveDomain(domain) {
    this.activeDomain = domain;
  }

  getDomains() {
    const allData = this.data.topics.concat(this.data.services, this.data.nodes);
    const domains = this.Domains.getDomains(allData);

    if (!this.activeDomain) {
      this.setActiveDomain(domains[0]);
    }
    return domains;
  }

  isDomainToShow(domain) {
    if (this.setting.advanced) {
      return true;
    }
    return this.DomainsToShow.includes(domain);
  }

  hasFilteredDomains(advanced) {
    return _.some(_.map(this.getDomains(), dom => this.Domains.filterAdvanced(dom, advanced)));
  }

  getGlobalParameters() {
    return this.Domains.getGlobalParameters(this.data.parameters);
  }

  resetData() {
    this.data = {
      rosout: [],
      topics: [],
      nodes: [],
      parameters: [],
      services: [],
    };
  }

  newRosConnection() {
    if (isConnected || this.setting === angular.isUndefined) {
      return;
    }

    if (ros) {
      ros.close(); // Close old connection
      ros = false;
      return;
    }

    ros = new ROSLIB.Ros({ url: `ws://${this.setting.address}:${this.setting.port}` });

    ros.on('connection', () => {
      this.onConnected();
      isConnected = true;
      this.isConnected = isConnected;
    });

    ros.on('error', () => {
      isConnected = false;
      this.isConnected = isConnected;
    });

    ros.on('close', () => {
      isConnected = false;
      this.isConnected = isConnected;
    });
  }

  onConnected() {
    // wait a moment until ROS is loaded and initialized
    this.$timeout(() => {
      this.loadData();

      this.setConsole();
      if (this.setting.battery) {
        this.setBattery();
      }
    }, 1000); // [ms]
  }

  // Setup of console (in the right sidebar)
  setConsole() {
    const consoleTopic = new ROSLIB.Topic({
      ros,
      name: this.setting.log,
      messageType: 'rosgraph_msgs/Log',
    });
    consoleTopic.subscribe((message) => {
      const nameArray = message.name.split('/');
      const d = new Date((message.header.stamp.secs * 1E3) + (message.header.stamp.nsecs * 1E-6));

      message.abbr = (nameArray.length > 1) ? nameArray[1] : message.name;

      // String formatting of message time and date
      function addZero(i) { return i < 10 ? `0${i}` : `${i}`; }
      message.dateString = `${addZero(d.getHours())}:
      ${addZero(d.getMinutes())}:
      ${addZero(d.getSeconds())}.
      ${addZero(d.getMilliseconds())}`;
      this.data.rosout.unshift(message);

      if (this.data.rosout.length > this.maxConsoleEntries) {
        this.data.rosout.pop();
      }
    });
  }

  // Setup battery status
  setBattery() {
    const batteryTopic = new ROSLIB.Topic({
      ros,
      name: this.setting.batteryTopic,
      messageType: 'std_msgs/Float32',
    });
    batteryTopic.subscribe((message) => {
      this.batteryStatus = message.data;
    });
  }

  // Load structure, all data, parameters, topics, services, nodes...
  loadData() {
    this.resetData();

    ros.getTopics((topics) => { // Topics now has topics and types arrays
      angular.forEach(topics.topics, (name) => {
        this.data.topics.push({ name });

        ros.getTopicType(name, (type) => {
          _.findWhere(this.data.topics, { name }).type = type;
        });
      });
    });

    ros.getServices((services) => {
      angular.forEach(services, (name) => {
        this.data.services.push({ name });

        ros.getServiceType(name, (type) => {
          _.findWhere(this.data.services, { name }).type = type;
        });
      });
    });

    ros.getParams((params) => {
      angular.forEach(params, (name) => {
        const param = new ROSLIB.Param({ ros, name });
        this.data.parameters.push({ name });

        param.get((value) => {
          _.findWhere(this.data.parameters, { name }).value = value;
        });
      });
    });

    ros.getNodes((nodes) => {
      angular.forEach(nodes, (name) => {
        this.data.nodes.push({ name });
      });
    });
  }

  runBash() {
    const postConfig = {
      url: '/api/killros',
      method: 'POST',
      data: 'killros',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded; charset=UTF-8' },
    };
    this.$http.post(postConfig.url, postConfig.data, postConfig.headers);
    // .success((data) => {
    //  angular.log.info(data);
    // });
  }
}

angular.module('roscc').component('ccControl', {
  templateUrl: 'app/control/control.html',
  controller: ControlController,
});
