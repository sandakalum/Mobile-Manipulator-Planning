package irvs.tms.smartpal_teleop_android;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import smartpal_control.sp_control;
import smartpal_control.sp_controlRequest;
import smartpal_control.sp_controlResponse;;

public class SrvClient extends AbstractNodeMain {

  ServiceClient<sp_controlRequest, sp_controlResponse> serviceClient;
  
  int  isCalled=0;
  byte result;
  
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("smartpal_teleop_android");
  }
  
  @Override
  public void onStart(final ConnectedNode connectedNode) {
    try {
      serviceClient = connectedNode.newServiceClient("sp5_control", sp_control._TYPE);
    } catch (ServiceNotFoundException e) {
      throw new RosRuntimeException(e);
    }   
  }
  
  public byte sendSrv(byte unit, byte cmd, double x_mm, double y_mm, double t_deg) {
    int cnt=0;
    isCalled = 0;
    
    sp_controlRequest request = serviceClient.newMessage();
    
    double []arg = new double[3];
    arg[0] = x_mm;
    arg[1] = y_mm;
    arg[2] = t_deg;
    
    request.setUnit(unit);
    request.setCmd(cmd);
    request.setArg(arg);
    
    serviceClient.call(request, new ServiceResponseListener<sp_controlResponse>() {
      @Override
      public void onSuccess(sp_controlResponse response) {
        isCalled = 1;
        result   = response.getResult();
      }

      @Override
      public void onFailure(RemoteException e) {
        throw new RosRuntimeException(e);
      }
    });
    while(isCalled!=1 && cnt<10){  
      try {
        Thread.sleep(500);  // 500ms * 10 = 5sec
      } catch (InterruptedException e) { }
      cnt++;
    }
    if(cnt>=10) result = -1;
    return result;
  }
}