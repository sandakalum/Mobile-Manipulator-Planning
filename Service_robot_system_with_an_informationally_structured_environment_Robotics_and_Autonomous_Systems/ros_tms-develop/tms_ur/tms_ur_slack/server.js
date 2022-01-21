const { RTMClient } = require('@slack/client');
const token = process.env.SLACK_TOKEN;

const rtm = new RTMClient(token);
const roslib = require('roslib');

const ros = new roslib.Ros({
    url: "ws://localhost:9090"
});
const cnannel = 'channel_id';


ros.on('connection', () => {
    console.log("Launch websocket server");
});

ros.on('error', error => {
    console.log(error);
});

ros.on('close', () => {
    console.log("Connection to websocket server was closed");
});

const slack_msg = new roslib.Topic({
    ros: ros,
    name: '/slack_msg',
    messageType: 'std_msgs/String'
});

const slack_server = new roslib.Service({
    ros: ros,
    name: '/slack_srv',
    serviceType: 'tms_ur_slack/slack_srv',
});

slack_server.advertise((req, res) => {
    console.log(req);
    rtm.sendMessage(req.data,channel);
    res = 1;
});

rtm.start();

rtm.on('message', event => {
    console.log(event);
//    if(event.subtype != 'channel_join' && event.subtype != 'channel_leave' && event.subtype != 'message_deleted'){
    if(!('subtype' in event) && !('bot_id' in event)){    
        let msg = new roslib.Message({
            data: event.text
        });
        slack_msg.publish(msg);
    }
    //let rep_message = robot_message;
    //rtm.sendMessage(rep_message, event.channel);
    //rtm.sendMessage(｀<@${event.user}> ${rep_message}｀, event.channel);
});