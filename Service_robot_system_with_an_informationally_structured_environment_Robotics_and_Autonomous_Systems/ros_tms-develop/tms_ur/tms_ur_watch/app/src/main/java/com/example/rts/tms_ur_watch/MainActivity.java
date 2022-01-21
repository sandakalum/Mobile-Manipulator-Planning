package com.example.rts.tms_ur_watch;

import android.content.Intent;
import android.graphics.Color;
import android.graphics.Typeface;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.support.wearable.activity.WearableActivity;
import android.support.wearable.view.BoxInsetLayout;
import android.util.Log;
import android.view.View;
import android.widget.TextView;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import static android.R.attr.data;
import static android.graphics.Typeface.BOLD;
import static android.graphics.Typeface.DEFAULT_BOLD;

public class MainActivity extends RosActivity {
    private rosNode rosnode;

    public MainActivity() {
        super("tms_ur_watch","tms_ur_watch",URI.create("http://192.168.4.170:11311"));
        Log.d("ROS","const");
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        Log.d("ROS", "init");
        WifiManager manager = (WifiManager)getSystemService(WIFI_SERVICE);
        WifiInfo info = manager.getConnectionInfo();
        int ipAdr = info.getIpAddress();
        String ipaddress = String.format("%d.%d.%d.%d",(ipAdr>>0)&0xff, (ipAdr>>8)&0xff, (ipAdr>>16)&0xff, (ipAdr>>24)&0xff);
//        rosnode = new rosNode();
        rosnode = (rosNode) this.getApplication();
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(ipaddress);

        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(rosnode, nodeConfiguration);

        Log.d("ROS", "initEND"+ipaddress);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
//        setContentView(R.layout.activity_main);

        Intent intent = new Intent(this,Watch.class);
        startActivity(intent);
        Log.d("ROS", "oncreate");
    }

    @Override
    protected void onResume() {
        super.onResume();
    }

    @Override
    protected void onPause() {
        super.onPause();
    }
}

//public class MainActivity extends RosActivity implements SensorEventListener {
//    private SensorManager mSensorManager;
//    private TextView textView;
//    public float hb = 100.0f;
//    private rosNode rosnode;
//
//    public MainActivity() {
//        super("tms_ur_watch","tms_ur_watch",URI.create("http://192.168.4.170:11311"));
//        Log.d("main","const");
//    }
//
//    @Override
//    protected void init(NodeMainExecutor nodeMainExecutor) {
//        Log.d("ROS", "init");
//        WifiManager manager = (WifiManager)getSystemService(WIFI_SERVICE);
//        WifiInfo info = manager.getConnectionInfo();
//        int ipAdr = info.getIpAddress();
//        String ipaddress = String.format("%d.%d.%d.%d",(ipAdr>>0)&0xff, (ipAdr>>8)&0xff, (ipAdr>>16)&0xff, (ipAdr>>24)&0xff);
//        rosnode = new rosNode();
//        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(ipaddress);
//
//        nodeConfiguration.setMasterUri(getMasterUri());
//        nodeMainExecutor.execute(rosnode, nodeConfiguration);
//
//        Log.d("ROS", "initEND"+ipaddress);
//    }
//
//    @Override
//    protected void onCreate(Bundle savedInstanceState) {
//        super.onCreate(savedInstanceState);
//        setContentView(R.layout.activity_main);
//
//        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
//        Sensor sensor = mSensorManager.getDefaultSensor(Sensor.TYPE_HEART_RATE);
//        mSensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_NORMAL);
//        textView = (TextView) findViewById(R.id.text);
//        textView.setText("測定中");
//
//
//
//        Log.d("debug", "oncreate");
//    }
//
//    @Override
//    protected void onResume() {
//        super.onResume();
//    }
//
//    @Override
//    protected void onPause() {
//        super.onPause();
//        mSensorManager.unregisterListener(this);
//    }
//
//    @Override
//    public void onSensorChanged(SensorEvent event) {
//        if (event.sensor.getType() == Sensor.TYPE_HEART_RATE && event.values[0] != 0) {
//            hb = event.values[0];
//            textView.setText("♥" + (int) hb);
//            Log.d("heartrate", "" + (int) hb);
//            rosnode.publishHBR((int) hb);
//        }
//    }
//
//    @Override
//    public void onAccuracyChanged(Sensor sensor, int accuracy) {
//        Log.d("accuracy_change", "" + accuracy);
//    }
//}


//public class MainActivity extends WearableActivity implements SensorEventListener {
//
//    private SensorManager mSensorManager;
//    private TextView textView;
//    public float hb = 100.0f;
////    private ROSclass ros;
//
//
//    @Override
//    protected void onCreate(Bundle savedInstanceState) {
//        super.onCreate(savedInstanceState);
//        setContentView(R.layout.activity_main);
//        setAmbientEnabled();
//
//        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
//        Sensor sensor = mSensorManager.getDefaultSensor(Sensor.TYPE_HEART_RATE);
//        mSensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_NORMAL);
//        textView = (TextView) findViewById(R.id.text);
//        textView.setText("測定中");
//
//        Log.d("debug", "oncreate");
//    }
//
//    @Override
//    protected void onResume() {
//        super.onResume();
//    }
//
//    @Override
//    protected void onPause() {
//        super.onPause();
//        mSensorManager.unregisterListener(this);
//    }
//
//    @Override
//    public void onSensorChanged(SensorEvent event) {
//        if (event.sensor.getType() == Sensor.TYPE_HEART_RATE && event.values[0] != 0) {
//            hb = event.values[0];
//            textView.setText("♥" + (int) hb);
//            Log.d("heartrate", "" + (int) hb);
//        }
//    }
//
//    @Override
//    public void onAccuracyChanged(Sensor sensor, int accuracy) {
//        Log.d("accuracy_change", "" + accuracy);
//    }
//
//}