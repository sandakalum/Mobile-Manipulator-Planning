package irvs.tms.smartpal_teleop_android;

import android.os.Bundle;

import android.widget.Button;
import android.widget.TextView;
import android.view.View.OnClickListener;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class MainActivity extends RosActivity {

  private SrvClient srvClient;
  private TextView  tvResult;
  private Button    btnF,btnB,btnR,btnL,btnRT,btnLT,btnS;
  
  public MainActivity() {
    super("smartpal_teleop_android", "smartpal_teleop_android");
  }

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main);
    
    tvResult  = (TextView) findViewById(R.id.tvResult);
    btnF      = (Button)   findViewById(R.id.btnF);
    btnB      = (Button)   findViewById(R.id.btnB);
    btnR      = (Button)   findViewById(R.id.btnR);
    btnL      = (Button)   findViewById(R.id.btnL);
    btnRT     = (Button)   findViewById(R.id.btnRT);
    btnLT     = (Button)   findViewById(R.id.btnLT);
    btnS      = (Button)   findViewById(R.id.btnS);
  }

  @Override
  protected void init(NodeMainExecutor nodeMainExecutor) {
    srvClient = new SrvClient();
    
    NodeConfiguration nodeConfiguration 
      = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),getMasterUri());
    
    nodeConfiguration.setMasterUri(getMasterUri());
    nodeMainExecutor.execute(srvClient, nodeConfiguration);  
    
    btnF.setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(android.view.View v) {
        byte srvResult = srvClient.sendSrv((byte)1,(byte)16,100,0,0);
        tvResult.setText("" + srvResult);
      }
    });
    
    btnB.setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(android.view.View v) {
        byte srvResult = srvClient.sendSrv((byte)1,(byte)16,-100,0,0);
        tvResult.setText("" + srvResult);
      }
    });
    
    btnR.setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(android.view.View v) {
        byte srvResult = srvClient.sendSrv((byte)1,(byte)16,0,-100,0);
        tvResult.setText("" + srvResult);
      }
    });
    
    btnL.setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(android.view.View v) {
        byte srvResult = srvClient.sendSrv((byte)1,(byte)16,0,100,0);
        tvResult.setText("" + srvResult);
      }
    });
    
    btnRT.setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(android.view.View v) {
        byte srvResult = srvClient.sendSrv((byte)1,(byte)16,0,0,-10);
        tvResult.setText("" + srvResult);
      }
    });
    
    btnLT.setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(android.view.View v) {
        byte srvResult = srvClient.sendSrv((byte)1,(byte)16,0,0,10);
        tvResult.setText("" + srvResult);
      }
    });
    /*
    btnS.setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(android.view.View v) {
        byte srvResult = srvClient.sendSrv((byte)1,(byte)16,100,0,0);
        tvResult.setText("" + srvResult);
      }
    });
    */
  }
}
