package com.github.irvs.ros_tms.tms_ur.tms_ur_mimamorukun;

import android.os.Handler;
import android.os.Message;
import android.util.Log;

import org.ros.concurrent.CancellableLoop;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import tms_msg_db.TmsdbGetData;
import tms_msg_db.TmsdbGetDataRequest;
import tms_msg_db.TmsdbGetDataResponse;

/**
 * Created by kazuto on 15/03/24.
 */
public class db_reader_client extends AbstractNodeMain {
    private String TAG = "db_reader_client";
    private String srv_name = "/tms_db_reader";

    private ServiceClient<TmsdbGetDataRequest, TmsdbGetDataResponse> dbClient;
    private Handler handler;
    public Pose current_pose = new Pose();

    private int id = 2007;     //Mimamorukun
    private int sensor = 3001; //Vicon

    public class Pose {
        public double x;
        public double y;
        public double yaw;
    }

    public db_reader_client(Handler handler) {
        this.handler = handler;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("tms_ur_mimamorukun/db_reader_client");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        Log.d(TAG, "onStart");
        String status;
        try {
            dbClient = connectedNode.newServiceClient(srv_name, TmsdbGetData._TYPE);
            status = "connected";
        } catch (ServiceNotFoundException e) {
            Log.d(TAG, "ServiceNotFoundException");
            status = "failed to connect";
            //throw new RosRuntimeException(e);
        }
        Message msg = handler.obtainMessage(TmsUrMimamorukun.DBREADER_STATUS, status);
        handler.sendMessage(msg);

        if (dbClient != null) {
            connectedNode.executeCancellableLoop(new CancellableLoop() {
                final TmsdbGetDataRequest req = dbClient.newMessage();

                @Override
                protected void setup() {
                    Log.d(TAG, "setup");
                    req.getTmsdb().setId(id); // Mimamorukun ID
                    req.getTmsdb().setSensor(sensor); // 3001: Vicon, 3501: kalman filter data
                }

                @Override
                protected void loop() throws InterruptedException {
                    Log.d(TAG, "loop()");
                    dbClient.call(req, new ServiceResponseListener<TmsdbGetDataResponse>() {
                        @Override
                        public void onSuccess(TmsdbGetDataResponse res) {
                            Log.d(TAG, "onSuccess()");
                            final int idx = 0;
                            if (!res.getTmsdb().isEmpty()) {
                                current_pose.x = res.getTmsdb().get(idx).getX() / 1000;
                                current_pose.y = res.getTmsdb().get(idx).getY() / 1000;
                                current_pose.yaw = res.getTmsdb().get(idx).getRy();
                                Message msg = handler.obtainMessage(TmsUrMimamorukun.UPDATE_POSITION, "update msg");
                                handler.sendMessage(msg);
                            }
                        }

                        @Override
                        public void onFailure(RemoteException e) {
                            Log.d(TAG, "onFailure");
                        }
                    });

                    Thread.sleep(100); // 100msec
                }
            });
        }
    }

    @Override
    public void onShutdown(Node node) {
        super.onShutdown(node);
    }
}
