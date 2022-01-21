package com.example.rts.tms_ur_watch;

import android.app.Application;
import android.util.Log;

import org.ros.concurrent.CancellableLoop;
import org.ros.internal.message.RawMessage;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;


import std_msgs.Int32;

/**
 * Created by rts on 10/23/17.
 */

public class rosNode extends Application implements NodeMain {
    private Publisher<std_msgs.Int32> HBRPublisher;
    private Publisher<std_msgs.String> voicePublisher;

    @Override
    public void onStart(ConnectedNode node){
        HBRPublisher = node.newPublisher("/watch_HBR",std_msgs.Int32._TYPE);
        voicePublisher = node.newPublisher("/watch_msg",std_msgs.String._TYPE);
    }

    @Override
    public GraphName getDefaultNodeName(){
        return GraphName.of("tms_ur_watch");
    }

    @Override
    public void onShutdown(Node arg0) {
    }

    @Override
    public void onShutdownComplete(Node arg0) {
    }

    @Override
    public void onError(Node arg0,Throwable arg1){
    }

    public void publishHBR(int rate){
        if(HBRPublisher != null){
            std_msgs.Int32 msg = HBRPublisher.newMessage();
            msg.setData(rate);
            HBRPublisher.publish(msg);
            Log.d("publish",""+msg);
        }
    }

    public void publishVoice(String str){
        if(voicePublisher != null){
            std_msgs.String msg = voicePublisher.newMessage();
            msg.setData(str);
            voicePublisher.publish(msg);
            Log.d("publish",""+msg);
        }
    }
}
