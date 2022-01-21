/*
 * PROJECT: NyARToolkit for Android SDK
 * --------------------------------------------------------------------------------
 * This work is based on the original ARToolKit developed by
 *   Hirokazu Kato
 *   Mark Billinghurst
 *   HITLab, University of Washington, Seattle
 * http://www.hitl.washington.edu/artoolkit/
 *
 * NyARToolkit for Android SDK
 *   Copyright (C)2010 NyARToolkit for Android team
 *   Copyright (C)2010 R.Iizuka(nyatla)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * For further information please contact.
 *  http://sourceforge.jp/projects/nyartoolkit-and/
 *
 * This work is based on the NyARToolKit developed by
 *  R.Iizuka (nyatla)
 *    http://nyatla.jp/nyatoolkit/
 *
 * contributor(s)
 *  Atsuo Igarashi
 */

package com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity;

import android.annotation.SuppressLint;
import android.app.AlertDialog;
import android.app.Dialog;
import android.app.ProgressDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.res.Configuration;
import android.content.res.Resources;
import android.hardware.Camera;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Message;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.OrientationEventListener;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup.LayoutParams;
import android.view.Window;
import android.view.WindowManager;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ListView;
import android.widget.TextView.BufferType;
import android.widget.Toast;
import android.widget.ViewFlipper;

import com.android.camera.CameraHardwareException;
import com.android.camera.CameraHolder;
import com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.data.Com;
import com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.data.Com.COMMAND;
import com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.data.TmsdbObject;
import com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.data.TmsdbObjectListAdapter;
import com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.ftp.FtpClient;
import com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.srv.TmsdbGetDataNode;
import com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.widget.Popup;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.net.URI;
import java.util.ArrayList;
import java.util.List;

//public class NyARToolkitAndroidActivity extends Activity implements View.OnClickListener, SurfaceHolder.Callback {
@SuppressLint("InflateParams")
public class TmsUrArviewer extends RosActivity implements View.OnClickListener,
SurfaceHolder.Callback , SensorEventListener/*,min3d.interfaces.ISceneController*/ {

	public TmsUrArviewer() {
		super("tms_ur_arviewer","tms_ur_arviewer", URI.create("http://192.168.4.170:11311/"));
		Log.v("ROS", "Const");
	}

	//Setting関連==================================
	private static int USERID;
	private static String FTPIP,FTPUSR,FTPPASS;

	private final int UID = 1;
	private final String FIP = "192.168.4.170";
	private final String FU = "rtsftp";
	private final String FP = "tmsftp";
	@SuppressLint("SdCardPath")
	private final String savePath = Environment.getExternalStorageDirectory().getPath() + "/Android/data/jp.androidgroup.nyartoolkit/";
	private final String saveDir = "images";
	//=============================================

	//Debug用
	private int d_cnt = 0;
	private void debugLog(String str){
		Log.v("DEBUG",d_cnt + ":" + str);
		d_cnt++;
	}

	//FTP
	private FtpClient fclient;
	private ProgressDialog dialog;

	//物品リスト描画
	//	private LayoutInflater inflater;
	//	private View view;
	//	private LinearLayout ll;
	private TmsdbObject currentObject;

	public static final String TAG = "NyARToolkitAndroid";

	private static final int CROP_MSG = 1;
	private static final int FIRST_TIME_INIT = 2;
	private static final int RESTART_PREVIEW = 3;
	private static final int CLEAR_SCREEN_DELAY = 4;

	public static final int SHOW_LOADING = 6;
	public static final int HIDE_LOADING = 7;

	private static final int SCREEN_DELAY = 2 * 60 * 1000;

	private android.hardware.Camera.Parameters mParameters;

	private OrientationEventListener mOrientationListener;
	private int mLastOrientation = 0;


	private android.hardware.Camera mCameraDevice;
	private SurfaceView mSurfaceView;
	private SurfaceHolder mSurfaceHolder = null;
	private boolean mStartPreviewFail = false;

	private boolean mPreviewing;
	private boolean mPausing;
	private boolean mFirstTimeInitialized;

	private Handler mHandler = new MainHandler();

	private PreviewCallback mPreviewCallback = new PreviewCallback();

	private com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.ARToolkitDrawer arToolkitDrawer = null;


	private Handler m_Handler;

	//	private FrameLayout frame;
	//	private SurfaceView mySurfaceView;
	private com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.MyView2 m_View;
	private LinearLayout controller;
	//	private CircleView CView;

	private Button btnRenew,/*btnSend,*/btnSetting,btnARInit,btnTest;
	private final int Renew=0,Send=1,Setting=2,ARInit = 3,Test=4;
	private com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.InitARPatt patt;
	private boolean isInitializedROS = false;
	private AlertDialog listDialog;


	/** This Handler is used to post message back onto the main thread of the application */
	private class MainHandler extends Handler {
		@SuppressWarnings("deprecation")
		@Override
		public void handleMessage(Message msg) {
			switch (msg.what) {
			case RESTART_PREVIEW: {
				restartPreview();
				break;
			}

			case CLEAR_SCREEN_DELAY: {
				getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
				break;
			}

			case FIRST_TIME_INIT: {
				initializeFirstTime();
				break;
			}

			case SHOW_LOADING: {
				showDialog(DIALOG_LOADING);
				break;
			}
			case HIDE_LOADING: {
				try {
					dismissDialog(DIALOG_LOADING);
					removeDialog(DIALOG_LOADING);
				} catch (IllegalArgumentException e) {
				}
				break;
			}
			}
		}
	}

	private static final int DIALOG_LOADING = 0;

	@SuppressWarnings("deprecation")
	@Override
	protected Dialog onCreateDialog(int id) {
		switch (id) {
		case DIALOG_LOADING: {
			ProgressDialog dialog = new ProgressDialog(this);
			dialog.setMessage("Loading ...");
			// dialog.setIndeterminate(true);
			dialog.setCancelable(false);
			dialog.getWindow().setFlags
			(WindowManager.LayoutParams.FLAG_BLUR_BEHIND,
					WindowManager.LayoutParams.FLAG_BLUR_BEHIND);
			return dialog;
		}
		default:
			return super.onCreateDialog(id);
		}
	}

	public static int roundOrientation(int orientationInput) {
		//		Log.d("roundOrientation", "orientationInput:" + orientationInput);
		int orientation = orientationInput;
		if (orientation == -1)
			orientation = 0;

		orientation = orientation % 360;
		int retVal;
		if (orientation < (0*90) + 45) {
			retVal = 0;
		} else if (orientation < (1*90) + 45) {
			retVal = 90;
		} else if (orientation < (2*90) + 45) {
			retVal = 180;
		} else if (orientation < (3*90) + 45) {
			retVal = 270;
		} else {
			retVal = 0;
		}

		return retVal;
	}

	// Snapshots can only be taken after this is called. It should be called
	// once only. We could have done these things in onCreate() but we want to
	// make preview screen appear as soon as possible.
	private void initializeFirstTime() {
		if (mFirstTimeInitialized) return;

		Log.d(TAG, "initializeFirstTime");
		debugLog("initializeFirstTime");
		// Create orientation listenter. This should be done first because it
		// takes some time to get first orientation.
		mOrientationListener =
				new OrientationEventListener(this) {
			@Override
			public void onOrientationChanged(int orientation) {
				// We keep the last known orientation. So if the user
				// first orient the camera then point the camera to
				// floor/sky, we still have the correct orientation.
				if (orientation != ORIENTATION_UNKNOWN) {
					orientation += 90;
				}
				orientation = roundOrientation(orientation);
				if (orientation != mLastOrientation) {
					mLastOrientation = orientation;
				}
			}
		};
		mOrientationListener.enable();

		mFirstTimeInitialized = true;

		changeGLSurfaceViewState();
	}

	// If the activity is paused and resumed, this method will be called in
	// onResume.
	private void initializeSecondTime() {
		Log.d(TAG, "initializeSecondTime");
		debugLog("initializeSecondTime");
		// Start orientation listener as soon as possible because it takes
		// some time to get first orientation.
		mOrientationListener.enable();

		changeGLSurfaceViewState();
	}

	/**
	 * Callback interface used to deliver copies of preview frames as they are displayed.
	 */
	private final class PreviewCallback
	implements android.hardware.Camera.PreviewCallback {

		@Override
		public void onPreviewFrame(byte [] data, Camera camera) {
			Log.d(TAG, "PreviewCallback.onPreviewFrame");

			if (mPausing) {
				return;
			}

			if(data != null) {
				Log.d(TAG, "data exist");

				if (arToolkitDrawer != null)
					arToolkitDrawer.draw(data);

			} else {
				try {
					// The measure against over load.
					Thread.sleep(500);
				} catch (InterruptedException e) {
					;
				}
			}
			restartPreview();
		}
	}

	/** Called with the activity is first created. */
	@SuppressWarnings("deprecation")
	@Override
	public void onCreate(Bundle savedInstanceState) {
		Log.v(TAG, "onCreate");
		super.onCreate(savedInstanceState);
		debugLog("onCreate");
		m_Handler = new Handler();

		//		_updateSceneHander = new Handler();


		m_View = new com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.MyView2(this);

		//		CView = new CircleView(this);
		//		frame = (FrameLayout)findViewById(R.id.frame);
		requestWindowFeature(Window.FEATURE_PROGRESS);

		Window win = getWindow();
		win.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		win.addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);

		//設定
		loadPref("USERID","FTPIP","FTPUSR","FTPPASS");

		setContentView(R.layout.main);

		//ボタンのセット
		int BUTTON_WIDTH = 200;
		controller = new LinearLayout(this);
		controller.setOrientation(LinearLayout.HORIZONTAL);
		//				controller.setGravity(Gravity.BOTTOM);
		controller.setGravity(Gravity.CENTER);
		// 登録ボタン
		btnRenew = new Button(this);
		btnRenew.setText("RENEW");
		btnRenew.setOnClickListener(this);
		btnRenew.setWidth(BUTTON_WIDTH);
		btnRenew.setId(Renew);

		btnSetting = new Button(this);
		btnSetting.setText("SET");
		btnSetting.setOnClickListener(this);
		btnSetting.setWidth(BUTTON_WIDTH);
		btnSetting.setId(Setting);
		btnARInit = new Button(this);
		btnARInit.setText("ARInit");
		btnARInit.setOnClickListener(this);
		btnARInit.setWidth(BUTTON_WIDTH);
		btnARInit.setId(ARInit);
		btnTest = new Button(this);
		btnTest.setText("Test");
		btnTest.setOnClickListener(this);
		btnTest.setWidth(BUTTON_WIDTH);
		btnTest.setId(Test);
		controller.addView(btnRenew);

		controller.addView(btnSetting);
		controller.addView(btnARInit);
		controller.addView(btnTest);


		mSurfaceView = (SurfaceView) findViewById(R.id.camera_preview);
		//		mySurfaceView = (SurfaceView) findViewById(R.id.my_view);        
		//		mSurfaceView.setKeepScreenOn(true);
		//		mySurfaceView.setKeepScreenOn(true);
		LayoutParams layoutParam = new LayoutParams(mSurfaceView.getLayoutParams());
		addContentView(m_View,layoutParam);
		//		addContentView(CView,new LayoutParams(LayoutParams.WRAP_CONTENT, LayoutParams.WRAP_CONTENT));
		m_View.setKeepScreenOn(true);
		//		frame.addView(m_View,new LayoutParams(LayoutParams.WRAP_CONTENT, LayoutParams.WRAP_CONTENT));
		//		m_View.setKeepScreenOn(true);
		// don't set mSurfaceHolder here. We have it set ONLY within
		// surfaceChanged / surfaceDestroyed, other parts of the code
		// assume that when it is set, the surface is also set.
		SurfaceHolder holder = mSurfaceView.getHolder();
		holder.addCallback(this);
		holder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);
		debugLog("onCreate");
		slideInLeft = AnimationUtils.loadAnimation(this,R.anim.slide_in_left);
		slideInRight = AnimationUtils.loadAnimation(this,R.anim.slide_in_right);
		slideOutLeft = AnimationUtils.loadAnimation(this,R.anim.slide_out_left);
		slideOutRight = AnimationUtils.loadAnimation(this,R.anim.slide_out_right);

		//		ll = (LinearLayout)findViewById(R.id.objectLayout);
		//センサー
		mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
		debugLog("onCreate");
		Log.v(TAG, "onCreateEnd");
		addContentView(controller,new LayoutParams(LayoutParams.WRAP_CONTENT, LayoutParams.WRAP_CONTENT));
	}

	private void changeGLSurfaceViewState() {
		// If the camera resumes behind the lock screen, the orientation
		// will be portrait. That causes OOM when we try to allocation GPU
		// memory for the GLSurfaceView again when the orientation changes. So,
		// we delayed initialization of GLSurfaceView until the orientation
		// becomes landscape.
		Configuration config = getResources().getConfiguration();
		if (config.orientation == Configuration.ORIENTATION_LANDSCAPE
				&& !mPausing && mFirstTimeInitialized) {
			if (arToolkitDrawer == null) initializeARToolkitDrawer();
		} else 
			if (arToolkitDrawer != null) {
				finalizeARToolkitDrawer();
			}
	}

	private void initializeARToolkitDrawer() {
		Log.d(TAG,"initARToolkitDrawer");
		// init ARToolkit.
		debugLog("initializeART");
		if (isInitializedROS){
			//			if(data.sendFurnitures()!=0){
			debugLog("setCameraPara");
			InputStream camePara = getResources().openRawResource(R.raw.camera_para);
			//			if(data.getFurnitureArray() != null){
			ArrayList<InputStream> patt = new ArrayList<InputStream>();
			debugLog("read pattern");
			File fp = new File(savePath + "patt");
			//フォルダが無い場合は作成
			if(!fp.exists()) fp.mkdirs();
			String[] names = new String[fp.list().length+1];

			//dummypatternのセット
			patt.add(getResources().openRawResource(R.raw.dummy));
			names[0] = "";

			for(int i=0;i<fp.list().length;i++){
				try {
					patt.add(new FileInputStream(fp.listFiles()[i]));
					names[i+1] = fp.list()[i];
				} catch (FileNotFoundException e) {
					Log.v("DEBUG",fp.list()[i] + "is Not Found");
					e.printStackTrace();
				}

			}

			debugLog("setWidth");
			int[] width = new int[patt.size()];
			for (int i = 0; i < patt.size(); i++) {
				width[i] = 80;
			}
			m_View.setNames(names);
			arToolkitDrawer = new com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.ARToolkitDrawer(camePara, width, patt, m_View/*,CView*/);
		}

		//		}
		Log.d(TAG,"initARToolkitDrawerEnd");
		debugLog("initializeARTEND");
		//		}
	}

	private void finalizeARToolkitDrawer() {
		debugLog("finalizeART");
		arToolkitDrawer = null;
	}

	@Override
	protected void onDestroy() {
		Log.v(TAG, "onDestroy");
		super.onDestroy();
		debugLog("onDestroy");
	}

	@Override
	public void onStart() {
		Log.v(TAG, "onStart");
		super.onStart();
		debugLog("onStart");
	}

	@Override
	public void onStop() {
		Log.v(TAG, "onStop");
		super.onStop();
		debugLog("onStop");
	}

	@Override
	public void onClick(View v) {
		switch(v.getId()){
		case Renew:
			dialog = new ProgressDialog(com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.TmsUrArviewer.this);
			dialog.setProgressStyle(ProgressDialog.STYLE_SPINNER);
			dialog.setMessage("Download Image Files ...");
			dialog.show();
			//無理やり非同期処理
			AsyncTask<Void,Void,Void> FTPTask = new AsyncTask<Void,Void,Void>(){
				@Override
				protected Void doInBackground(Void... params){
					fclient = new FtpClient(FTPIP,FTPUSR,FTPPASS);
					fclient.ChangeDir("2D");//2Dフォルダへ
					fclient.ChangeDir("images");
					fclient.SetDLPass(savePath,saveDir);//DLフォルダ作成
					fclient.Refresh();//DLフォルダの中身を削除
					fclient.DownloadAll();//imageフォルダの中身を全てダウンロード
					fclient.close();//解放(ぶっちゃけいらない)
					return null;
				}
				@Override
				protected void onPostExecute(Void result){
					dialog.dismiss();
					//viewの更新
				}
			};
			//非同期処理実行
			FTPTask.execute();
			break;

		case Setting:
			LayoutInflater inflater 
			= LayoutInflater.from(com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.TmsUrArviewer.this);
			View view = inflater.inflate(R.layout.setting, null);

			//SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(MainActivity.this);

			final EditText uid = (EditText)view.findViewById(R.id.editUID);
			final EditText fip = (EditText)view.findViewById(R.id.editFtpIP);
			final EditText fu = (EditText)view.findViewById(R.id.editFtpUsr);
			final EditText fp = (EditText)view.findViewById(R.id.editFtpPass);

			uid.setText(String.valueOf(USERID), BufferType.NORMAL);
			fip.setText(FTPIP, BufferType.NORMAL);
			fu.setText(FTPUSR, BufferType.NORMAL);
			fp.setText(FTPPASS, BufferType.NORMAL);

			Log.v(TAG,"Start!!");

			new AlertDialog.Builder(com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.TmsUrArviewer.this)
			.setTitle("SETTING")
			.setIcon(R.drawable.setting2)
			.setView(view)
			.setPositiveButton(
					"APPLY", 
					new DialogInterface.OnClickListener() {          
						@Override
						public void onClick(DialogInterface dialog, int which) {
							USERID = Integer.valueOf(uid.getText().toString());
							FTPIP = fip.getText().toString();
							FTPUSR = fu.getText().toString();
							FTPPASS = fp.getText().toString();
							savePref("USERID","FTPIP","FTPUSR","FTPPASS");
							Log.v(TAG,"positive***");
							Log.v(TAG,"USERID:" + USERID);
							Log.v(TAG,"FTPIP:" + FTPIP);
							Log.v(TAG,"FTPUSR:" + FTPUSR);
							Log.v(TAG,"FTPPASS:" + FTPPASS);
						}
					})
					.setNeutralButton("DEFAULT", new DialogInterface.OnClickListener() {          
						@Override
						public void onClick(DialogInterface dialog, int which) {
							USERID = UID;
							FTPIP = FIP;
							FTPUSR = FU;
							FTPPASS = FP;
							savePref("USERID","FTPIP","FTPUSR","FTPPASS");
						}
					})
					.setNegativeButton("CANCEL",null)
					.show();
			break;
		case ARInit://ARマーカをダウンロードする
			dialog = new ProgressDialog(com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.TmsUrArviewer.this);
			dialog.setProgressStyle(ProgressDialog.STYLE_SPINNER);
			dialog.setMessage("AR Marker Initialize ...");
			dialog.show();
			//無理やり非同期処理
			AsyncTask<Void,Void,Void> ARInitTask = new AsyncTask<Void,Void,Void>(){
				@Override
				protected Void doInBackground(Void... params){
					//全ファイルのダウンロード
					com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.InitARPatt iap = new com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.InitARPatt(FTPIP,FTPUSR,FTPPASS);
					iap = null;

					arToolkitDrawer = null;
					initializeARToolkitDrawer();

					return null;
				}
				@Override
				protected void onPostExecute(Void result){
					dialog.dismiss();
					//viewの更新
				}
			};
			//非同期処理実行
			ARInitTask.execute();
			break;
		case Test:
			//			popup.draw(data.getObjectArray().get(1));
			Toast.makeText(this, "Test", Toast.LENGTH_LONG).show();
			break;

		}
	}

	//プリファレンスの取得・保存
	private void savePref(String taguid,String tagfip, String tagfu, String tagfp){
		SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
		pref.edit().putInt(taguid,USERID).commit();
		pref.edit().putString(tagfip,String.valueOf(FTPIP)).commit();
		pref.edit().putString(tagfu,String.valueOf(FTPUSR)).commit();
		pref.edit().putString(tagfp,String.valueOf(FTPPASS)).commit();
	}

	private void loadPref(String taguid,String tagfip, String tagfu, String tagfp){
		SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
		USERID = pref.getInt(taguid, UID);
		FTPIP = pref.getString(tagfip, FIP);
		FTPUSR = pref.getString(tagfu, FU);
		FTPPASS = pref.getString(tagfp, FP);
	}

	@Override
	public void onResume() {
		super.onResume();
		Log.d(TAG, "onResume");
		debugLog("onResume");
		mPausing = false;

		// Start the preview if it is not started.
		if (!mPreviewing && !mStartPreviewFail && (mSurfaceHolder != null)) {
			try {
				startPreview();
			} catch (Exception e) {
				showCameraErrorAndFinish();
				return;
			}
		}

		if (mSurfaceHolder != null) {
			// If first time initialization is not finished, put it in the
			// message queue.
			if (!mFirstTimeInitialized) {
				mHandler.sendEmptyMessage(FIRST_TIME_INIT);
			} else {
				initializeSecondTime();
			}
		}
		keepScreenOnAwhile();

		rutinHandler = new RutinHandler();
		rutinHandler.sleep(0);

		initSensor();
	}

	@Override
	protected void onPause() {
		Log.d(TAG, "onPause");
		super.onPause();
		debugLog("onPause");
		mPausing = true;
		stopPreview();
		// Close the camera now because other activities may need to use it.
		closeCamera();
		resetScreenOn();
		changeGLSurfaceViewState();

		if (mFirstTimeInitialized) {
			mOrientationListener.disable();
		}

		// Remove the messages in the event queue.
		mHandler.removeMessages(RESTART_PREVIEW);
		mHandler.removeMessages(FIRST_TIME_INIT);

		rutinHandler = null;

		finalizeSensor();
		super.onPause();
	}

	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		Log.v("ROS", "onActivityResult");
		super.onActivityResult(requestCode, resultCode, data);
		debugLog("onActivityResult");
		switch (requestCode) {
		case CROP_MSG: {
			Log.v("ROS", "CROP_MSG");
			Intent intent = new Intent();
			if (data != null) {
				Bundle extras = data.getExtras();
				if (extras != null) {
					intent.putExtras(extras);
				}
			}
			setResult(resultCode, intent);
			finish();
			break;
		}
		}
	}

	@Override
	public boolean onTouchEvent(MotionEvent event) {
		switch (event.getAction()) {
		case MotionEvent.ACTION_DOWN:
			break;

		case MotionEvent.ACTION_MOVE:
			break;

		case MotionEvent.ACTION_UP:
			break;
		}
		return true;
	}

	@Override
	public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {
		Log.d(TAG, "surfaceChanged");
		debugLog("surfaceChanged");

		// Make sure we have a surface in the holder before proceeding.
		if (holder.getSurface() == null) {
			Log.d(TAG, "holder.getSurface() == null");
			return;
		}

		// We need to save the holder for later use, even when the mCameraDevice
		// is null. This could happen if onResume() is invoked after this
		// function.
		mSurfaceHolder = holder;

		// The mCameraDevice will be null if it fails to connect to the camera
		// hardware. In this case we will show a dialog and then finish the
		// activity, so it's OK to ignore it.
		if (mCameraDevice == null) {

			/*
			 * To reduce startup time, we start the preview in another thread.
			 * We make sure the preview is started at the end of surfaceChanged.
			 */
			Thread startPreviewThread = new Thread(new Runnable() {
				public void run() {
					try {
						mStartPreviewFail = false;
						startPreview();
					} catch (Exception e) {
						// In eng build, we throw the exception so that test tool
						// can detect it and report it
						if ("eng".equals(Build.TYPE)) {
							throw new RuntimeException(e);
						}
						mStartPreviewFail = true;
					}
				}
			});
			startPreviewThread.start();

			// Make sure preview is started.
			try {
				startPreviewThread.join();
				if (mStartPreviewFail) {
					showCameraErrorAndFinish();
					return;
				}
			} catch (InterruptedException ex) {
				// ignore
			}
		}

		// Sometimes surfaceChanged is called after onPause.
		// Ignore it.
		if (mPausing || isFinishing()) return;

		// If first time initialization is not finished, send a message to do
		// it later. We want to finish surfaceChanged as soon as possible to let
		// user see preview first.
		if (!mFirstTimeInitialized) {
			mHandler.sendEmptyMessage(FIRST_TIME_INIT);
		} else {
			initializeSecondTime();
		}
	}

	@Override
	public void surfaceCreated(SurfaceHolder holder) {
		debugLog("SurfaceCreated");
	}

	@Override
	public void surfaceDestroyed(SurfaceHolder holder) {
		stopPreview();
		mSurfaceHolder = null;
		debugLog("SurfaceDestroyed");
	}

	private void closeCamera() {
		if (mCameraDevice != null) {
			CameraHolder.instance().release();
			mCameraDevice = null;
			mPreviewing = false;
		}
	}

	private void ensureCameraDevice() throws CameraHardwareException {
		if (mCameraDevice == null) {
			mCameraDevice = CameraHolder.instance().open();
		}
	}

	private void showCameraErrorAndFinish() {
		Resources ress = getResources();
		com.android.camera.Util.showFatalErrorAndFinish(com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.TmsUrArviewer.this,
				ress.getString(R.string.camera_error_title),
				ress.getString(R.string.cannot_connect_camera));
	}

	public void restartPreview() {
		Log.d(TAG, "restartPreview");
		try {
			startPreview();
		} catch (CameraHardwareException e) {
			showCameraErrorAndFinish();
			return;
		}
	}

	private void setPreviewDisplay(SurfaceHolder holder) {
		try {
			mCameraDevice.setPreviewDisplay(holder);
		} catch (Throwable ex) {
			closeCamera();
			throw new RuntimeException("setPreviewDisplay failed", ex);
		}
	}

	private void startPreview() throws CameraHardwareException {
		if (mPausing || isFinishing()) return;

		ensureCameraDevice();

		// If we're previewing already, stop the preview first (this will blank
		// the screen).
		// FIXME: don't stop for avoiding blank screen.
		//        if (mPreviewing) stopPreview();

		setPreviewDisplay(mSurfaceHolder);
		if (!mPreviewing)
			setCameraParameters();

		mCameraDevice.setOneShotPreviewCallback(mPreviewCallback);

		try {
			Log.v(TAG, "startPreview");
			mCameraDevice.startPreview();
		} catch (Throwable ex) {
			closeCamera();
			throw new RuntimeException("startPreview failed", ex);
		}
		mPreviewing = true;
		//		mStatus = IDLE;
	}

	private void stopPreview() {
		if (mCameraDevice != null && mPreviewing) {
			Log.v(TAG, "stopPreview");
			mCameraDevice.setOneShotPreviewCallback(null);
			mCameraDevice.stopPreview();
		}
		mPreviewing = false;
	}

	private void setCameraParameters() {
		mParameters = mCameraDevice.getParameters();

		mParameters.setPreviewSize(320, 240);
		//		mParameters.setRotation(90);
		mCameraDevice.setParameters(mParameters);

	}

	private void resetScreenOn() {
		mHandler.removeMessages(CLEAR_SCREEN_DELAY);
		getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
	}

	private void keepScreenOnAwhile() {
		mHandler.removeMessages(CLEAR_SCREEN_DELAY);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		mHandler.sendEmptyMessageDelayed(CLEAR_SCREEN_DELAY, SCREEN_DELAY);
	}


	public void PostRunnable(Runnable runnable) {
		m_Handler.post(runnable);
	}

	/*******************************r
	 * ROS_INIT
	 *******************************/

	private TmsdbGetDataNode data;
	//	private final String className = "TmsdbGetDataNode";
	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {
		Log.v("ROS", "init");
		debugLog("ROSinit");
		data = new TmsdbGetDataNode();
		NodeConfiguration nodeConfiguration 
		= NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),getMasterUri());

		nodeConfiguration.setMasterUri(getMasterUri());
		nodeMainExecutor.execute(data, nodeConfiguration);
		isInitializedROS = true;
		Log.v("ROS", "initEND");
		debugLog("ROSinitEnd");
	}


	private int lastFurnitureIndex=0;
	private int dis_count = 0;

	//物品リストを表示

	private void popupList(){
		//何も見つかっていない
		if(m_View.getFurnitureIndex()==0){
			if(listDialog != null){//リストダイアログがインスタンス化されている
				if(listDialog.isShowing()&&dis_count>2){//表示されている
					listDialog.dismiss();
					dis_count = 0;
				}
				else{
					dis_count++;
				}
			}
		}
		else{//何か見つかっている
			//前回と同じIndexの時
			if(lastFurnitureIndex == m_View.getFurnitureIndex()){
				if(listDialog!=null){
					if(!listDialog.isShowing()) listDialog.show();
				}
			}
			//前回と違うインデックス
			else{
				if(listDialog!=null) listDialog.dismiss();
				lastFurnitureIndex = m_View.getFurnitureIndex();
				String name = m_View.getFurnitureName();
				int fIndex = 0;
				data.sendFurniture();
				for(TmsdbObject furniture : data.getFurnitureArray()){
					if(!name.equals(furniture.getName())){
						fIndex++;
					}
					else{
						break;
					}
				}
				debugLog("name:" + name +"**index:" + fIndex);
				debugLog("numOfF:" + data.getFurnitureArray().size() + "**" + "nowid:" + data.getFurnitureArray().get(fIndex).getId());
				//			data.sendInfo(nowId);
				//			if(data.getObject()!=null) Toast.makeText(this, data.getObject().getName(), Toast.LENGTH_LONG).show();

				if(data.getFurnitureArray().get(fIndex).getId() != 0){
					currentObject = null;
					currentObject = new TmsdbObject(data.getFurnitureArray().get(fIndex));
					//				drawWindow();
					/*
					 * 
					 * drawWindaw
					 * 
					 * 
					 * */

					debugLog("drawWindow:start");
					int size = data.sendBelongObject(currentObject);
					debugLog("drawWindow:getBelongObjectSize::" + size);

					/************************************************
					 * リストビューの表示
					 ************************************************/
					LayoutInflater inflater = LayoutInflater.from(com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.TmsUrArviewer.this);
					View dView = inflater.inflate(R.layout.popupformat,null);

					ListView objLayout = (ListView)dView.findViewById(R.id.listView);

					ArrayList<TmsdbObject> objs = new ArrayList<TmsdbObject>();
					for(TmsdbObject object : data.getObjectArray()){
						objs.add(object);
					}

					TmsdbObjectListAdapter adapter = new TmsdbObjectListAdapter(this, 0, objs, savePath + saveDir);

					objLayout.setAdapter(adapter);

					final ArrayList<TmsdbObject> params = objs;

					objLayout.setOnItemClickListener(new OnItemClickListener(){

						@Override
						public void onItemClick(AdapterView<?> adapter, View v,
								int position, long id) {
							//					Toast.makeText(v.getContext(), "pos:"+ position + "_id:" + id, Toast.LENGTH_LONG).show();
							Popup popup = new Popup(v.getContext(), savePath + saveDir);
							popup.draw(params.get(position));
						}

					});

					listDialog = null;
					listDialog = new AlertDialog.Builder(com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.TmsUrArviewer.this)
					.setTitle(currentObject.getName())
					.setIcon(R.drawable.icon)
					.setView(dView)
					//				.setPositiveButton("TEST",new DialogInterface.OnClickListener() {          
					//					@Override
					//					public void onClick(DialogInterface dialog, int which) {
					//						
					//					}
					//				})
					//			.setNegativeButton("CLOSE",null)
					.show();

					/************************************************
					 * リストビューの表示終了
					 ************************************************/



				}
				else Toast.makeText(this, "data is not exist!", Toast.LENGTH_LONG).show();


			}
		}
	}

	ViewFlipper vf;
	private Animation slideInLeft,slideInRight,slideOutLeft,slideOutRight;
	Com pre_com;
	int com_num = 0;

	private void popupObjectFlipper(){

		//何か見つかっている
		if(m_View.getFurnitureIndex()!=0){
			//前回と同じIndexの時
			//			if(lastFurnitureIndex == m_View.getFurnitureIndex()){
			//				if(listDialog!=null){
			//					if(!listDialog.isShowing()) listDialog.show();
			//				}
			//			}
			//前回と違うインデックス
			//			else{
			if(listDialog==null){
				//			if(listDialog!=null) listDialog.dismiss();
				//			lastFurnitureIndex = m_View.getFurnitureIndex();
				String name = m_View.getFurnitureName();
				int fIndex = 0;
				data.sendFurniture();
				for(TmsdbObject furniture : data.getFurnitureArray()){
					if(!name.equals(furniture.getName())){
						fIndex++;
					}
					else{
						break;
					}
				}
				debugLog("name:" + name +"**index:" + fIndex);
				debugLog("numOfF:" + data.getFurnitureArray().size() + "**" + "nowid:" + data.getFurnitureArray().get(fIndex).getId());
				//			data.sendInfo(nowId);
				//			if(data.getObject()!=null) Toast.makeText(this, data.getObject().getName(), Toast.LENGTH_LONG).show();

				if(data.getFurnitureArray().get(fIndex).getId() != 0){
					currentObject = null;
					currentObject = new TmsdbObject(data.getFurnitureArray().get(fIndex));
					//				drawWindow();
					/*
					 * 
					 * drawWindaw
					 * 
					 * 
					 * */

					debugLog("drawWindow:start");
					int size = data.sendBelongObject(currentObject);
					debugLog("drawWindow:getBelongObjectSize::" + size);

					//					//vfはonCreate で予めレイアウトをセットしておく

					vf = null;

					vf = new ViewFlipper(com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.TmsUrArviewer.this);
					vf.setLayoutParams(new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.MATCH_PARENT));

					for(TmsdbObject object : data.getObjectArray()){
						Popup p = new Popup(vf.getContext(), savePath + saveDir);
						p.setView(object);
						vf.addView(p.getDatailView());
					}

					listDialog = null;
					listDialog = new AlertDialog.Builder(com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.TmsUrArviewer.this)
					.setTitle(currentObject.getName())
					.setIcon(R.drawable.icon)
					.setView(vf)
					.show();

				}
				//			}
			}}

		//コマンドの処理　n回以上同じコマンドが送られてきた場合に一致と判断それ以外はCENTER
		//同じコマンドは繰り返さない
		final int n1 = 3, n2 = 6;

		Com popup_com = new Com();
		if(pre_com == null) pre_com = new Com();
		if(command != null) popup_com.setCommand(command.getCommand());

		if(popup_com.getCommand().equals(pre_com.getCommand())) com_num++;
		else com_num = 0;

		if(com_num != n1&&com_num != n2){
			pre_com.setCommand(popup_com.getCommand());
			popup_com.setCommand(COMMAND.CENTER);
		}

		else{
			if(com_num==n1){
				if(popup_com.getCommand().equals(COMMAND.UP)||popup_com.getCommand().equals(COMMAND.DOWN)){
					pre_com.setCommand(popup_com.getCommand());
					popup_com.setCommand(COMMAND.CENTER);
				}
			}
			else{
				if(popup_com.getCommand().equals(COMMAND.LEFT)||popup_com.getCommand().equals(COMMAND.RIGHT)){
					pre_com.setCommand(popup_com.getCommand());
					popup_com.setCommand(COMMAND.CENTER);
				}
			}
		}
		//コマンド処理終了


		//commandの取得処理終了　最終的な命令はpopup_comに格納

//		debugLog(popup_com.getCommand().toString());

		//UIの処理
		if(listDialog!=null){
			if(listDialog.isShowing()){
				switch(popup_com.getCommand()){
				case CENTER:
					break;
				case LEFT:
					vf.setInAnimation(slideInRight);
					vf.setOutAnimation(slideOutLeft);
					vf.showNext();
					break;
				case RIGHT:
					vf.setInAnimation(slideInLeft);
					vf.setOutAnimation(slideOutRight);
					vf.showPrevious();
					break;
				case UP:
					listDialog.dismiss();
					listDialog = null;
					break;
				case DOWN:
					break;
				default:
					break;
				}
			}
		}



	}


	//センサ関連

	//センサマネージャ　onCreate でインスタンス化する
	private SensorManager mSensorManager;

	@Override
	public void onAccuracyChanged(Sensor arg0, int arg1) {

	}

	//センサの値取得時に呼ばれる Comクラスcomをpublicで定義してコマンド伝達

	float[] gyroValues = new float[3];
	float[] accValues = new float[3];

	float X = 0f,Y = 0f,Z = 0f;

	//とても適当な相補フィルタ(重み付け平均)係数
	final float compVal1 = 0.90f, compVal2 = 0.1f;
	//とても適当なセンサのしきい値 z軸は正or負
	final float tz=3f, ty=4f; //MOVERIO
//	final float tx=3f, ty=3f; //GS3

	Com command;

	@Override
	public void onSensorChanged(SensorEvent event) {
		if (event.accuracy == SensorManager.SENSOR_STATUS_UNRELIABLE) return;

		switch (event.sensor.getType()) {
		case Sensor.TYPE_GYROSCOPE:
			gyroValues = event.values.clone();
			break;
		case Sensor.TYPE_ACCELEROMETER:
			accValues = event.values.clone();
			break;
		}

		if(gyroValues != null && accValues != null){

			if(command == null) command = new Com();

			//とても適当な相補フィルタ
			Z = compVal1*(Z + gyroValues[0]) + compVal2*accValues[0];
			X = compVal1*(X + gyroValues[1]) + compVal2*accValues[1];
			Y = compVal1*(Y + gyroValues[2]) + compVal2*accValues[2];

			//値取得完了　ここからコマンド生成
			//MOVERIO用
			//LEFT Z +3 RIGHT Z -3
			//UP Y -3 DOWN Y +3

			if(Z >  tz) command.setCommand(COMMAND.LEFT);
			else if(Z < -tz) command.setCommand(COMMAND.RIGHT);
			else if(Y < -ty) command.setCommand(COMMAND.UP);
			else if(Y >  ty) command.setCommand(COMMAND.DOWN);
			else command.setCommand(COMMAND.CENTER);
			
//			//値取得完了　ここからコマンド生成
//			//GS3用
//			//LEFT X -6 RIGHT X 6
//			//UP Y 3 DOWN Y -3
//
//			if(X <  -tx) command.setCommand(COMMAND.LEFT);
//			else if(X > tx) command.setCommand(COMMAND.RIGHT);
//			else if(Y > ty) command.setCommand(COMMAND.DOWN);
//			else if(Y < -ty) command.setCommand(COMMAND.UP);
//			else command.setCommand(COMMAND.CENTER);

		}

	}

	//ここに使うセンサの設定を書く　onResumeで呼ぶ

	private boolean mIsGyroSensor, mIsAccSensor;

	private void initSensor(){
		List<Sensor> sensors = mSensorManager.getSensorList(Sensor.TYPE_ALL);
		for (Sensor sensor : sensors) {

			if( sensor.getType() == Sensor.TYPE_GYROSCOPE){
				mSensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_NORMAL);
				mIsGyroSensor = true;
			}

			if( sensor.getType() == Sensor.TYPE_ACCELEROMETER){
				mSensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_NORMAL);
				mIsAccSensor = true;
			}
		}
	}

	//onStopで呼ぶ（呼ばないと無駄に電池を消耗する）
	private void finalizeSensor(){
		mSensorManager.unregisterListener(this);
	}


	//定期処理ハンドラ

	private RutinHandler rutinHandler;

	private class RutinHandler extends Handler {
		@Override
		public void handleMessage(Message msg) {
			//処理記述
			if(command != null) debugLog(command.toString());
			//			popupList();
			popupObjectFlipper();
			if(m_View != null) debugLog("FurIndex:" + m_View.getFurnitureIndex());
			if (rutinHandler!=null) rutinHandler.sleep(100);  //3.
		}

		//スリープメソッド
		public void sleep(long delayMills) {
			//使用済みメッセージの削除
			removeMessages(0);
			sendMessageDelayed(obtainMessage(0),delayMills);  //4.
		}
	}

}






















