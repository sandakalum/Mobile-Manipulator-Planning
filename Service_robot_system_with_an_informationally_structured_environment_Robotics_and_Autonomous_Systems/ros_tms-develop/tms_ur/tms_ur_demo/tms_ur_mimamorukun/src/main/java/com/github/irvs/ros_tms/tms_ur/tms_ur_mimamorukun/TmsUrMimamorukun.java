package com.github.irvs.ros_tms.tms_ur.tms_ur_mimamorukun;

import android.content.Context;
import android.content.res.Configuration;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Rect;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.support.v4.app.ActionBarDrawerToggle;
import android.support.v4.widget.DrawerLayout;
import android.util.Log;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.InputStream;
import java.net.URI;

/**
 * Created by kazuto on 15/03/24.
 */
public class TmsUrMimamorukun extends RosActivity
{
    private String TAG = "tms_ur_mimamorukun";

    private LinearLayout debug_info;
    private TextView ontouch_info, rp_cmd_info, target_info, current_info;

    private TextView ros_master_uri, local_ip;
    private TextView rp_cmd_status, dbreader_status;

    private WcIcon wc_icon;
    private RoomMap room_map;

    private int adjust_mode = 0;
    private boolean in_motion = false;

    private ActionBarDrawerToggle drawerToggle;
    private DrawerLayout drawer;

    private Handler handler;

    private Button execute_button;
    private Switch debug_switch;
    private Switch calib_switch;

    //adjust_mode switch
    private RadioGroup mode_switch;
    private RadioButton radio_position;
    private RadioButton radio_orientation;

    //conditional tag for handler
    public static final int UPDATE_POSITION = 0;
    public static final int RP_CMD_RESULT = 1;
    public static final int RP_CMD_STATUS = 2;
    public static final int DBREADER_STATUS = 3;
    public static final int POSITION_SETTING = 0;
    public static final int ORIENTATION_SETTING = 1;

    //ros nodes
    private static URI master_uri = URI.create("http://192.168.4.170:11311");
    private rp_cmd_client rp_cmd_client;
    private db_reader_client db_reader_client;

    final Context context = this;

    //pose
    public class Pose {
        public double x;
        public double y;
        public double yaw;
    }

    //Wheelchair icon class
    public class WcIcon {
        public ImageView current, target;
        public float[] current_size = {0, 0};
        public float[] target_size = {0, 0};
        private Pose current_pose = new Pose();
        private Pose target_pose = new Pose();
        private float scale = (float)0.15;

        public WcIcon() {
            current = (ImageView)findViewById(R.id.current);
            target = (ImageView)findViewById(R.id.target);

            InputStream is = getResources().openRawResource(R.raw.wc_icon);
            Bitmap bm = BitmapFactory.decodeStream(is);
            current.setImageBitmap(bm);
            current.setScaleX(scale);
            current.setScaleY(scale);

            is = getResources().openRawResource(R.raw.wc_icon2);
            bm = BitmapFactory.decodeStream(is);
            target.setImageBitmap(bm);
            target.setScaleX(scale);
            target.setScaleY(scale);
            target.setRotation(-90);
        }

        public void init() {
            current_size[0] = current.getWidth();
            current_size[1] = current.getHeight();
            target_size[0] = target.getWidth();
            target_size[1] = target.getHeight();
            target.setVisibility(View.INVISIBLE);
        }
    }

    //room map icon class
    public class RoomMap {
        public ImageView map_image;
        private MapTouchListener mapTouchListener;
        public int[] map_offset = {0, 0};
        public float[] map_origin = {0, 0};
        public boolean calib_mode = false;
        public double map_scale = 0.0125;//4.4/(387.59 - 17.87) map to room

        public ImageView calib;
        public float[] calib_size = {0, 0};

        public RoomMap() {
            mapTouchListener = new MapTouchListener();
            map_origin[0] = (float)17.87;
            map_origin[1] = (float)626.2;
        }

        public void init() {
            if (!calib_mode) {
                Log.d("calib", "calib_mode:false");
                if (map_image != null) {
                    map_image = null;
                    calib.setVisibility(View.INVISIBLE);
                    calib = null;
                }
                map_image = (ImageView) findViewById(R.id.map_image);
                InputStream is = getResources().openRawResource(R.raw.map_image);
                Bitmap bm = BitmapFactory.decodeStream(is);
                map_image.setImageBitmap(bm);
            } else {
                Log.d("calib", "calib_mode:true");
                calib = (ImageView) findViewById(R.id.calib);
                InputStream is = getResources().openRawResource(R.raw.calib);
                Bitmap bm = BitmapFactory.decodeStream(is);
                calib.setImageBitmap(bm);
                calib.setVisibility(View.VISIBLE);

                if (map_image != null) {
                    map_image = null;
                }
                map_image = (ImageView) findViewById(R.id.map_image);
                is = getResources().openRawResource(R.raw.map_image_calib);
                bm = BitmapFactory.decodeStream(is);
                map_image.setImageBitmap(bm);
            }
            map_image.setOnTouchListener(mapTouchListener);
        }

        public void getSize() {
            map_image.getLocationOnScreen(map_offset);
            final Rect rect = new Rect();
            Window window = getWindow();
            window.getDecorView().getWindowVisibleDisplayFrame(rect);
            map_offset[1] -= rect.top; //the height of status bar
            Log.d(TAG, "statusbar height: " + rect.top);
            Log.d("CHECK", "map_offset: " + map_offset[0] + ", " + map_offset[1]);

            if (calib_mode) {
                calib_size[0] = calib.getWidth();
                calib_size[1] = calib.getHeight();
            }
        }

        //OnTouchListener for the map
        public class MapTouchListener implements View.OnTouchListener {
            private double touch_x, touch_y, orientation;

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                Log.d(TAG, "X:" + event.getX() + ",Y:" + event.getY());
                ontouch_info.setText("onTouch()\ngetX: " + event.getX() + "\ngetY: " + event.getY());
                touch_x = event.getX();
                touch_y = event.getY();
                if (!calib_mode) {
                    switch (adjust_mode) {
                        case POSITION_SETTING:
                            setTargetPosition();
                            switch (event.getAction()) {
                                case MotionEvent.ACTION_DOWN:
                                    this.drawTargetIcon();
                                    break;
                                case MotionEvent.ACTION_UP:
                                    break;
                                case MotionEvent.ACTION_MOVE:
                                    this.drawTargetIcon();
                                    break;
                                case MotionEvent.ACTION_CANCEL:
                                    break;
                            }
                            break;
                        case ORIENTATION_SETTING:
                            switch (event.getAction()) {
                                case MotionEvent.ACTION_DOWN:
                                    this.rotateTargetIcon();
                                    break;
                                case MotionEvent.ACTION_UP:
                                    break;
                                case MotionEvent.ACTION_MOVE:
                                    this.rotateTargetIcon();
                                    break;
                                case MotionEvent.ACTION_CANCEL:
                                    break;
                            }
                            break;
                    }
                } else {
                    switch (event.getAction()) {
                        case MotionEvent.ACTION_DOWN:
                            this.drawTargetIcon();
                            break;
                        case MotionEvent.ACTION_UP:
                            break;
                        case MotionEvent.ACTION_MOVE:
                            this.drawTargetIcon();
                            break;
                        case MotionEvent.ACTION_CANCEL:
                            break;
                    }
                }
                return true;
            }

            private void setTargetPosition() {
                wc_icon.target_pose.x = (touch_x - room_map.map_origin[0])*map_scale;
                wc_icon.target_pose.y = (room_map.map_origin[1] - touch_y)*map_scale;
            }

            private void drawTargetIcon() {
                if (!calib_mode) {
                    wc_icon.target.setVisibility(View.VISIBLE);
                    wc_icon.target.setX((float) touch_x - wc_icon.target_size[0] / 2 + room_map.map_offset[0]);
                    wc_icon.target.setY((float) touch_y - wc_icon.target_size[1] / 2 + room_map.map_offset[1]);
                    showTargetInfo();
                } else {
                    calib.setX((float) touch_x - calib_size[0] + room_map.map_offset[0]);
                    calib.setY((float) touch_y + room_map.map_offset[1]);
                    target_info.setText("Left bottom\nx: " + String.format("%.2f[px]\n", touch_x - calib_size[0]) +
                        "y: " + String.format("%.2f[px]\n", touch_y + calib_size[1]));
                }
            }

            private void rotateTargetIcon() {
                touch_x -= wc_icon.target.getX() + wc_icon.target_size[0] / 2 - room_map.map_offset[0];
                touch_y -= wc_icon.target.getY() + wc_icon.target_size[1] / 2 - room_map.map_offset[1];
                touch_y *= -1;
                orientation = Math.atan2(touch_y, touch_x);
                wc_icon.target_pose.yaw = orientation*180/Math.PI;
                wc_icon.target.setRotation(-(float)wc_icon.target_pose.yaw-90);
                showTargetInfo();
            }

            private void showTargetInfo() {
                rp_cmd_info.setText("x: " + String.format("%.2f[m]\n", wc_icon.target_pose.x) +
                    "y: " + String.format("%.2f[m]\n", wc_icon.target_pose.y) +
                    "yaw: " + String.format("%.2f[deg]", wc_icon.target_pose.yaw));
                target_info.setText("x: " + String.format("%.2f[m]\n", wc_icon.target_pose.x) +
                    "y: " + String.format("%.2f[m]\n", wc_icon.target_pose.y) +
                    "yaw: " + String.format("%.2f[deg]", wc_icon.target_pose.yaw));
            }
        }
    }

    public TmsUrMimamorukun() {
        super("Mimamorukun Control", "Mimamorukun Control", master_uri);
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        // for debug
        ontouch_info = (TextView)findViewById(R.id.ontouch_info);
        target_info = (TextView)findViewById(R.id.target_info);
        current_info = (TextView)findViewById(R.id.current_info);
        debug_info = (LinearLayout)findViewById(R.id.debug_info);
        debug_info.setVisibility(View.INVISIBLE);

        rp_cmd_info = (TextView)findViewById(R.id.rp_cmd_info);
        ros_master_uri  = (TextView)findViewById(R.id.ros_master_uri);
        local_ip  = (TextView)findViewById(R.id.local_ip);
        rp_cmd_status = (TextView)findViewById(R.id.rp_cmd_status);
        dbreader_status = (TextView)findViewById(R.id.dbreader_status);

        // the icon of a wheelchair
        wc_icon = new WcIcon();

        // the map of tms room
        room_map = new RoomMap();
        room_map.init();

        // navigation drawer
        drawer = (DrawerLayout) findViewById(R.id.drawer_layout);
        drawerToggle = new ActionBarDrawerToggle(this, drawer,
            org.ros.android.android_10.R.mipmap.icon,
            R.string.drawer_open,
            R.string.drawer_close) {
            @Override
            public void onDrawerClosed(View drawerView) {
                Log.d(TAG, "onDrawerClosed");
                if (!in_motion) {
                    room_map.map_image.setOnTouchListener(room_map.mapTouchListener);
                }
            }

            @Override
            public void onDrawerOpened(View drawerView) {
                Log.d(TAG, "onDrawerOpened");
                room_map.map_image.setOnTouchListener(null);
            }

            @Override
            public void onDrawerSlide(View drawerView, float slideOffset) {
                super.onDrawerSlide(drawerView, slideOffset);
                Log.d(TAG, "onDrawerSlide : " + slideOffset);
                drawer.requestLayout();
            }

            @Override
            public void onDrawerStateChanged(int newState) {
                Log.i(TAG, "onDrawerStateChanged  new state : " + newState);
            }
        };
        drawer.setDrawerListener(drawerToggle);

        // handler
        handler = new Handler() {
            @Override
            public void handleMessage(Message msg) {
                super.handleMessage(msg);
                switch (msg.what) {
                    case UPDATE_POSITION: {
                        Log.d(TAG, "handleMessage/UPDATE_POSITION");
                        wc_icon.current_pose.x = db_reader_client.current_pose.x/room_map.map_scale + room_map.map_origin[0];
                        wc_icon.current_pose.y = room_map.map_origin[1] - db_reader_client.current_pose.y/room_map.map_scale;
                        wc_icon.current_pose.yaw = db_reader_client.current_pose.yaw;
                        current_info.setText("x: " + String.format("%.2f[m]\n", db_reader_client.current_pose.x) +
                            "y: " + String.format("%.2f[m]\n", db_reader_client.current_pose.y) +
                            "yaw: " + String.format("%.2f[deg]", db_reader_client.current_pose.yaw));
                        drawCurrentIcon();
                        break;
                    }
                    case RP_CMD_RESULT: {
                        Log.d(TAG, "handleMessage/RP_CMD_RESULT");
                        in_motion = false;
                        room_map.map_image.setOnTouchListener(room_map.mapTouchListener);
                        Toast.makeText(context, (String)msg.obj, Toast.LENGTH_SHORT).show();
                        break;
                    }
                    case RP_CMD_STATUS: {
                        rp_cmd_status.setText((String)msg.obj);
                        break;
                    }
                    case DBREADER_STATUS: {
                        dbreader_status.setText((String)msg.obj);
                        break;
                    }
                }
            }

            private void drawCurrentIcon() {
                wc_icon.current.setX((float) wc_icon.current_pose.x - wc_icon.current_size[0] / 2 + room_map.map_offset[0]);
                wc_icon.current.setY((float) wc_icon.current_pose.y - wc_icon.current_size[1] / 2 + room_map.map_offset[1]);
                wc_icon.current.setRotation(-(float)wc_icon.current_pose.yaw-90);
            }
        };

        // service execution
        execute_button = (Button)findViewById(R.id.execute_button);
        execute_button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
            Log.d(TAG,"sendRequest");
            drawer.closeDrawers();
            in_motion = true;
            (new Thread(new Runnable() {
                @Override
                public void run() {
                    rp_cmd_client.sendRequest(wc_icon.target_pose.x*1000, wc_icon.target_pose.y*1000, wc_icon.target_pose.yaw, true);
                }
            })).start();
            }
        });

        WifiManager wifiManager = (WifiManager)getSystemService(WIFI_SERVICE);
        WifiInfo wifiInfo = wifiManager.getConnectionInfo();
        int ip_i = wifiInfo.getIpAddress();
        String ip = ((ip_i >> 0) & 0xFF) + "." + ((ip_i >> 8) & 0xFF) + "." + ((ip_i >> 16) & 0xFF) + "." + ((ip_i >> 24) & 0xFF);
        local_ip.setText(ip);

        debug_switch = (Switch)findViewById(R.id.debug_switch);
        debug_switch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
            if (isChecked) {
                debug_info.setVisibility(View.VISIBLE);
            } else {
                debug_info.setVisibility(View.INVISIBLE);
            }
            }
        });

        calib_switch = (Switch)findViewById(R.id.calib_switch);
        calib_switch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
            if (isChecked) {
                wc_icon.target.setVisibility(View.INVISIBLE);
                wc_icon.current.setVisibility(View.INVISIBLE);
                mode_switch.setVisibility(View.INVISIBLE);
                room_map.calib_mode = true;
                room_map.init();
                room_map.getSize();
            } else {
                wc_icon.target.setVisibility(View.VISIBLE);
                wc_icon.current.setVisibility(View.VISIBLE);
                mode_switch.setVisibility(View.VISIBLE);
                room_map.calib_mode = false;
                room_map.init();
                room_map.getSize();
            }
            }
        });

        mode_switch = (RadioGroup)findViewById(R.id.mode_switch);
        radio_position = (RadioButton)findViewById(R.id.radio_position);
        radio_orientation = (RadioButton)findViewById(R.id.radio_orientation);
        mode_switch.check(radio_position.getId());
        mode_switch.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                if (-1 == checkedId) {
                    ;
                } else {
                    if (checkedId == radio_position.getId()) {
                        adjust_mode = POSITION_SETTING;
                    } else {
                        adjust_mode = ORIENTATION_SETTING;
                    }
                }
            }
        });
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        Log.d(TAG, "onWindowFocusChanged()");
        super.onWindowFocusChanged(hasFocus);

        room_map.getSize();
        wc_icon.init();
    }

    // region NavigationDrawer
    @Override
    protected void onPostCreate(Bundle savedInstanceState) {
        super.onPostCreate(savedInstanceState);
        drawerToggle.syncState();
    }

    @Override
    public void onConfigurationChanged(Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
        drawerToggle.onConfigurationChanged(newConfig);
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        if (drawerToggle.onOptionsItemSelected(item)) {
            return true;
        }
        return super.onOptionsItemSelected(item);
    }
    //endregion

    @Override
    protected void onStart() {
        super.onStart();
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration
            = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());

        ros_master_uri.setText(getMasterUri().toString());

        rp_cmd_client = new rp_cmd_client(handler);
        nodeMainExecutor.execute(rp_cmd_client, nodeConfiguration);

        db_reader_client = new db_reader_client(handler);
        nodeMainExecutor.execute(db_reader_client, nodeConfiguration);
    }
}
