$(function(){

// In this file you can include the rest of your app's specific main process
// code. You can also put them in separate files and require them here.
'use strict';
console.log("create a nodejs listener in the browser window process ??");
const rclnodejs = require('rclnodejs/index.js');
const {QoS} = rclnodejs;

rclnodejs.init().then(() => {
  console.log("created a nodejs listener successfully ?");
  const node = rclnodejs.createNode('subscription_example_node');

//  node.createSubscription('std_msgs/msg/String', 'topic', (msg) => {
//    console.log(`Received new message: ${typeof msg}`, msg);
//        $('#ros-id').text(msg.data);
////    $('#other-text').text(msg.data);
//  });

  node.createSubscription('l6470_msgs/msg/MultiPose', 'multipose', (msg) => {
    console.log(`Received new pose message: ${typeof msg}`, msg);
//        $('#ros-id').text(msg.data);
//    $('#other-text').text(msg.data);
  });

  rclnodejs.spin(node);
});

});
