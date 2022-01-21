'use strict';

const roslib = require('roslib');
//const profile_url = "https://api.line.me/v2/bot/profile/";
const ros = new roslib.Ros({
	url: 'ws://localhost:9090'
});

const ros_cloud = new roslib.Ros({
	url: 'ws://hogehoge'
	//rosbridge for remote server
});

ros.on('connection',function(){
	console.log('Connected to websocket server');
});

ros.on('error', function(){
	console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function(){
	console.log('Connection to websocket server closed');
});

ros_cloud.on('connection',function(){
	console.log('Connected to websocket server');
});

ros_cloud.on('error', function(){
	console.log('Error connecting to websocket server: ', error);
});

ros_cloud.on('close', function(){
	console.log('Connection to websocket server closed');
});


const LineMsgToTMS = new roslib.Topic({
	ros: ros,
	name: '/line_msg',
	messageType: 'std_msgs/String'
});

let listener = new roslib.Topic({
  ros:ros_cloud,
  name: '/line_msg',
  messageType: 'std_msgs/String'
});

listener.subscribe(function(message){
  //console.log('Recieved message on ' + listener.name + ':' + message.data);
  let val = new roslib.Message({
    data: message.data
  });
  LineMsgToTMS.publish(val);
});
