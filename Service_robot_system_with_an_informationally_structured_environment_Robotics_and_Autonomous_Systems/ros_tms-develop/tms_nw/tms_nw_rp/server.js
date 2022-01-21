const express = require("express");
const setTimeout =  require("timers").setTimeout;
const roslib = require("roslib");
const http = require("http");
const url = require('url');
const ipware = require('ipware');
const bodyParser = require("body-parser");
const app = express();
const ipw = ipware();


app.use(bodyParser.json());
app.use(bodyParser.urlencoded({extended: true}));
app.post("/rp", (req, res) => {
    const request_from = ((ipw.get_ip(req).clientIp).split(":")).pop();
    const local_ip = req.headers.host.split(":")[0]
    if(request_from != local_ip){
        res.json({
            "message":"This host has no authority to access this server"
        });
        return;
    }

    const remote_url = req.body.url;
    const room_name = req.body.room;
    const command = req.body.command;

    let req_service = "";
    let req_service_type = ""; 
    if(command == "robot_task"){
        req_service = req.body.service;
        req_service_type = req.body.service_type;
    }


    const ros_remote = new roslib.Ros({
        url: "ws://" + remote_url + ":9090"
    });
    ros_remote.on('connection', () => {
        console.log('Connected to websocket server');
    });
    
    ros_remote.on('error', error =>{
        console.log('Erro connecting to websocket server: ', error);
    });
    
    ros_remote.on('close', () => {
        console.log("Connection to websocket server was closed");
    });

    const get_id = new roslib.Service({
        ros: ros_remote,
        name: "get_id",
        serviceType: "tms_nw_api/get_id"
    });

    let get_id_req = new roslib.ServiceRequest({
        url: "http://" + local_ip
    });

    get_id.callService(get_id_req, id_res => {
        console.log(id_res);
        let anc_list = id_res.task_announce.split("$");
        let announce = "";
        let room_flag = 0;
        console.log(anc_list);
        for(let anc in anc_list){
            if(anc_list[anc] == "object"){
                announce += id_res.object_announce;
            }else if(anc_list[anc] == "robot"){
                announce += id_res.robot_announce;
            }else if(anc_list[anc] == "place"){
                announce += id_res.place_announce;
            }else if(anc_list[anc] == "user"){
                announce += id_res.user_announce;
            }else if(anc_list[anc] == "data"){
                announce += id_res.data;
            }else{
                announce += anc_list[anc];
                if (command == "search_object" || command == "robot_task"){
                    if(room_flag == 0){
                        room_flag += 1;
                    }
                    else if(room_flag == 1){
                        announce += room_name + "の";
                        room_flag = 2;
                    }
                }else if(room_flag == 0){
                    announce += room_name + "の"
                    room_flag = 2
                }
            }
        }
        console.log(announce);

        if(command =="robot_task"){
            const remote_task = new roslib.Service({
                ros: ros_remote,
                name: req_service,
                serviceType: req_service_type
            });

            const remote_req = new roslib.ServiceRequest({
                "task_id": id_res.task_id,
                "robot_id": id_res.robot_id,
                "object_id":id_res.object_id,
                "user_id": id_res.user_id,
                "place_id": id_res.place_id,
                "priority": 0
            });

            remote_task.callService(remote_req, service_res =>{
                console.log(service_res);
                ros_remote.close();
            })

        }

        res.status(200).json({
            "message":"OK",
            "announce":announce
        });
        
        return;


});
    });

app.listen(5000);