package com.github.irvs.ros_tms.tms_ur.tms_ur_refrigerator;

import android.util.Log;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import tms_msg_rs.rs_home_appliancesRequest;
import tms_msg_rs.rs_home_appliancesResponse;

/**
 * RecognitionClient node
 * Receiving the recognized result, sends requests to the ROS service.
 */
public class RecognitionClient extends AbstractNodeMain {
    private final String TAG = "recognition_client";
    private final String srv_name = "/refrigerator_controller";
    private ServiceClient<rs_home_appliancesRequest, rs_home_appliancesResponse> serviceClient;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("tms_ur_refrigerator/recognition_client");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        try {
            serviceClient = connectedNode.newServiceClient(srv_name, tms_msg_rs.rs_home_appliances._TYPE);
        } catch (ServiceNotFoundException e) {
            //throw new RosRuntimeException(e3);
        }
    }

    /**
     * Sends the recognized text to server
     */
    public void sendRequest(int num) {
        final tms_msg_rs.rs_home_appliancesRequest request = serviceClient.newMessage();

        /**
         * Set the parameter of the service
         */
        request.setId(2009);
        request.setService(num);

        /**
         * Call the service, "ref_demo".
         */
        Log.i("RecognitionClient","Call service");
        serviceClient.call(request, new ServiceResponseListener<rs_home_appliancesResponse>() {
            @Override
            public void onSuccess(final tms_msg_rs.rs_home_appliancesResponse srvResponse) {
                Log.i(TAG, "Succeeded to call service");
            }
            @Override
            public void onFailure(RemoteException e) {
                Log.i(TAG, "Failed to call service");
                throw new RosRuntimeException(e);
            }
        });
    }

}