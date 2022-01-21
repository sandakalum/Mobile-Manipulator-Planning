const express = require("express");
//const ipfilter = require("express-ipfilter").IpFilter;
const bodyParser = require("body-parser");
const app = express();
const request = require("request");
const url = require('url');
const ipware = require('ipware');
const ipw = ipware();

const fs = require("fs");
const url_path = "resources/tms.json";

let tms_reciever_port = ":3000/post";

const tms_list = [];
const tms_url_list = [];
function getUrlList(filepath){
    let urlList = JSON.parse(fs.readFileSync(filepath, 'utf8'));

    urlList.tms.forEach(data => {
        tms_list.push(data);
        tms_url_list.push(data.url);
    });

}
getUrlList(url_path);

console.log(url_path);

console.log(tms_list);

app.use(bodyParser.json());
app.use(bodyParser.urlencoded({extended: true}));

app.post("/rms_svr", (req, res) => {
    let tms_sender= "http://";
    const tms_sender_ip = ((ipw.get_ip(req).clientIp).split(":")).pop();
    tms_sender += tms_sender_ip; 
    let tmp_tms_list = [];
    console.log("Http request from: " + tms_sender);

    const words = req.body.words;
    console.log(req.body);

    if(tms_url_list.indexOf(tms_sender) == -1){
        res.json({
            "message":"This host has no authority to access this server"
        });
        return;
    }else{
        tmp_tms_list = tms_list.filter((val) => {
            if(val.url != tms_sender){
                return val;
            }
        })
//         tmp_tms_list = tms_list;
    }
    if('uri' in req.body){
        tms_sender = req.body.uri;
    }

    const search_tms = (tms_url, tms_name, skype_id) => {
        return new Promise((resolve, reject) =>{
            const headers = {
                'Content-Type':'application/json'
            }
            const options = {
                url: tms_url,
                headers: headers,
                json: {
                    words,
                    url: tms_sender   
                }
            }

            request.post(options, function(error, response, body){
                if(error){
                    resolve(0);
                    return false;
                }
                if (!error && response.body.message == "OK"){
                    console.log(response.body);
                    const response_obj = {
                        "message":"OK",
                        "hostname":response.request.uri.hostname,
                        "name": tms_name,
                        "skype_id": skype_id,
                        /*
                        "service_id":{
                            "robot_id":response.body.service_id.robot_id,
                            "task_id":response.body.service_id.task_id,
                            "user_id":response.body.service_id.user_id,
                            "object_id":response.body.service_id.object_id,
                            "place_id":response.body.service_id.place_id,   
                        }
                        */
                    }
                    resolve(response_obj);
                    return true;
                }else if(!error && response.body.message == "OK_nested"){
                    console.log(response.body);
                    const response_obj = {
                        "message":"OK",
                        "hostname":response.body.uri,
                        "name": tms_name,
                        "skype_id": skype_id,
                    }
                    resolve(response_obj);
                    return true;

                }else{
                    console.log('error: '+response.body.message +" @ "+ tms_url);
                    resolve(0);
                    return false;
                }
            });
        });
    };
    

    let promises = [];
    for(let i = 0; i < tmp_tms_list.length; i++){
        promises.push(search_tms(tmp_tms_list[i].url + tms_reciever_port, tmp_tms_list[i].room, tmp_tms_list[i].skype_id));
    }
    Promise.all(promises).then(function(results){
        for(let i = 0; i < tmp_tms_list.length; i++){
            if(!results[i] == 0){
                res.json(results[i]);
                break;
            }else if(i == tmp_tms_list.length - 1){
                res.json({"message":"Could not find them in all tms"});
            }
        }
    })


});

app.listen(4000);