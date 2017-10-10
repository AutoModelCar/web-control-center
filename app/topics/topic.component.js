class TopicController {
  constructor($scope, $http, Settings, Quaternions) {
    this.$scope = $scope;
    this.$http = $http;
    this.setting = Settings.get();
    this.Quaternions = Quaternions;

    this.isSubscribing = false;
  }

  $onInit() {
    this.roslibTopic = new ROSLIB.Topic({
      ros,
      name: this.topic.name,
      messageType: this.topic.type,
    });

    const path = 'app/topics/';
    this.isDefault = true;

    // Check if file exists
    const fileName = `${path}${this.topic.type}.html`;
    this.$http.get(fileName).then((result) => {
      if (result.data) {
        this.fileName = fileName;
        this.isDefault = false;
      }
    });
    if (!this.fileName) {
      this.fileName = `${path}default.html`;
    }
  }

  toggleSubscription(data) {
    if (!data) {
      this.roslibTopic.subscribe((message) => {
        this.message = message;
        if (this.isDefault) {
          this.message = angular.toJson(message);
        }
      });
    } else {
      this.roslibTopic.unsubscribe();
    }
    this.isSubscribing = !data;
  }

  publishMessage(input, isJSON) {
    const data = isJSON ? angular.fromJson(input) : input;
    const message = new ROSLIB.Message(data);
    this.roslibTopic.publish(message);
  }

  loadLaserScan() {
    /* eslint-disable no-new */

    // Create the main viewer.
    const viewer = new ROS3D.Viewer({
      divID: 'viewer',
      width: 640,
      height: 480,
      background: '#444444',
      antialias: true,
    });

    // Setup a client to listen to TFs.
    const tfClient = new ROSLIB.TFClient({
      ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
      fixedFrame: '/laser',
    });

    new ROS3D.LaserScan({
      ros,
      tfClient,
      rootObject: viewer.scene,
      topic: '/scan',
      color: 0x0077FF,
    });
  }
}

angular.module('roscc').component('ccTopic', {
  bindings: { topic: '=' },
  template: '<ng-include src="$ctrl.fileName"></ng-include>',
  controller: TopicController,
});

