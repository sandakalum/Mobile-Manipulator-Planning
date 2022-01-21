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

/*
何か聞きたいことがあれば
eisen.wachs@gmail.com
大石　哲朗(2014年度修了)
まで
 */

package com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity;

import android.annotation.SuppressLint;
import android.app.AlertDialog;
import android.app.Dialog;
import android.app.ProgressDialog;
import android.content.ActivityNotFoundException;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.res.Configuration;
import android.content.res.Resources;
import android.graphics.Color;
import android.hardware.Camera;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Message;
import android.preference.PreferenceManager;
import android.speech.RecognizerIntent;
import android.speech.tts.TextToSpeech;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.OrientationEventListener;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup.LayoutParams;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.TextView.BufferType;
import android.widget.Toast;

import com.android.camera.CameraHardwareException;
import com.android.camera.CameraHolder;
import com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.data.TmsdbObject;
import com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.data.TmsdbObjectListAdapter;
import com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.ftp.FtpClient;
import com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.srv.TmsdbGetDataNode;
import com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.tag.TagAnalyzer;

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
import java.util.Locale;

//非推奨無視
//面倒くさいので無視してるけど
//今後Androidのバージョンが上がった場合に
//切られる可能性があるので互換の関数に置き換える必要あり
@SuppressWarnings("deprecation")
//LayoutInflaterのViewGroupの引数nullを無視
//これ自体は動作保障上、得に問題はない
@SuppressLint("InflateParams")
public class TmsUrSanmoku extends RosActivity implements View.OnClickListener,
SurfaceHolder.Callback,TextToSpeech.OnInitListener{

	public TmsUrSanmoku() {
		super("tms_ur_commander", "tms_ur_commander", URI.create("http://192.168.4.170:11311/"));
		Log.v("ROS", "Const");
	}

	//Setting関連==================================
	private static int USERID;
	private static String FTPIP,FTPUSR,FTPPASS;
	private static String MSQLIP,MSQLUSR,MSQLPASS;

	private final int UID = 1;
	private final String FIP = "192.168.4.161";
	private final String FU = "tmsftp";
	private final String FP = "tmsftp";

	private final String MIP = "192.168.4.124";
	private final String MU = "android";
	private final String MP = "pass";

	@SuppressLint("SdCardPath")
	private final String savePath = Environment.getExternalStorageDirectory().getPath() + "/Android/data/irvs.tms.urclientactivity/";
	private final String saveDir = "images";
	//=============================================

	//Debug用
	private int d_cnt = 0;
	private void debugLog(String str){
		Log.v("DEBUG",d_cnt + ":" + str);
		d_cnt++;
	}

	//FTP&MSQL
	private FtpClient fclient;
	private ProgressDialog dialog;
	private EditText txtInput;

	public static final String TAG = "TmsUrSanmoku";

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

	private com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.ARToolkitDrawer arToolkitDrawer = null;


	private Handler m_Handler;

	private com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.MyView2 m_View;

	private Button btnRenew,btnSetting,btnARInit,btnVoice,btnTxt;
	private final int Renew=0,Send=1,Setting=2,ARInit = 3,Send2=4;

	private boolean isInitializedROS = false;
	
	//ARパターン登録用
	//FTPまわりは実装が変なので要改善
	com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.InitARPatt patt;

	//音声関連
	private final int REQUEST_CODE = 10;

	private TextToSpeech    tts;//音声合成

	/** This Handler is used to post message back onto the main thread of the application */
	private class MainHandler extends Handler {
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
				Log.d(TAG, "OrientationChanged");
				if (orientation != ORIENTATION_UNKNOWN) {
					orientation += 90;
				}
				orientation = roundOrientation(orientation);
				if (orientation != mLastOrientation) {
					mLastOrientation = orientation;
				}
				Log.d(TAG, "OrientationChanged2");
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
	@Override
	public void onCreate(Bundle savedInstanceState) {
		Log.v(TAG, "onCreate");
		super.onCreate(savedInstanceState);
		debugLog("onCreate");
		m_Handler = new Handler();

		m_View = new com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.MyView2(this);

		requestWindowFeature(Window.FEATURE_PROGRESS);

		Window win = getWindow();
		win.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		win.addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);

		//設定
		loadPref("USERID","FTPIP","FTPUSR","FTPPASS","MSQLIP","MSQLUSR","MSQLPASS");

		setContentView(R.layout.main);

		// 登録ボタン
		btnRenew = (Button)findViewById(R.id.button4);
		btnRenew.setOnClickListener(this);
		btnRenew.setId(Renew);
		btnSetting = (Button)findViewById(R.id.button3);
		btnSetting.setOnClickListener(this);
		btnSetting.setId(Setting);
		btnARInit = (Button)findViewById(R.id.button5);
		btnARInit.setOnClickListener(this);
		btnARInit.setId(ARInit);
		btnVoice = (Button)findViewById(R.id.button1);
		btnVoice.setOnClickListener(this);
		btnVoice.setId(Send);
		btnTxt = (Button)findViewById(R.id.button2);
		btnTxt.setOnClickListener(this);
		btnTxt.setId(Send2);

		txtInput = (EditText)findViewById(R.id.editText1);
		sublayout = (LinearLayout)findViewById(R.id.sublayout);

		detectingObjs = new ArrayList<TmsdbObject>();

		// TextToSpeechオブジェクトの生成
		tts = new TextToSpeech(this, this);

		tts.setSpeechRate(1f);
		tts.setPitch(1f);

		mSurfaceView = (SurfaceView) findViewById(R.id.camera_preview);
		mSurfaceView.setBackgroundColor(Color.TRANSPARENT);
		mSurfaceView.setZOrderOnTop(true);


		LayoutParams layoutParam = new LayoutParams(mSurfaceView.getLayoutParams());
		addContentView(m_View,layoutParam);
		m_View.setKeepScreenOn(true);
		// don't set mSurfaceHolder here. We have it set ONLY within
		// surfaceChanged / surfaceDestroyed, other parts of the code
		// assume that when it is set, the surface is also set.
		SurfaceHolder holder = mSurfaceView.getHolder();
		holder.addCallback(this);
		holder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);
		debugLog("onCreate");
		new com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.tag.TagAnalyzer(MSQLIP,MSQLUSR,MSQLPASS);
		debugLog("onCreate");
		Log.v(TAG, "onCreateEnd");
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
			debugLog("setCameraPara");
			InputStream camePara = getResources().openRawResource(R.raw.camera_para);
			ArrayList<InputStream> patt = new ArrayList<InputStream>();
			debugLog("read pattern");
			File fp = new File(savePath + "patt");
			//フォルダが無い場合は作成
			if(!fp.exists()) fp.mkdirs();
			String[] names = new String[fp.list().length+1];

			//ダミーパターンのセット
			//なんだか一番初めに登録したマーカに認識が偏るので
			//何にも対応しないダミーパターンを登録してる
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
			arToolkitDrawer = new com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.ARToolkitDrawer(camePara, width, patt, m_View/*,CView*/);
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
		if (null != tts) {
			// TextToSpeechのリソースを解放する
			tts.shutdown();
		}
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
			dialog = new ProgressDialog(com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.TmsUrSanmoku.this);
			dialog.setProgressStyle(ProgressDialog.STYLE_SPINNER);
			dialog.setMessage("Download Image Files ...");
			dialog.show();
			//無理やり非同期処理
			AsyncTask<Void,Void,Void> FTPTask = new AsyncTask<Void,Void,Void>(){
				@Override
				protected Void doInBackground(Void... params){
					fclient = new FtpClient(FTPIP,FTPUSR,FTPPASS);
					if(fclient.IsConnected()){
						fclient.ChangeDir("2D");//2Dフォルダへ
						fclient.ChangeDir("images");
						fclient.SetDLPass(savePath,saveDir);//DLフォルダ作成
						fclient.Refresh();//DLフォルダの中身を削除
						fclient.DownloadAll();//imageフォルダの中身を全てダウンロード
						fclient.close();//解放(ぶっちゃけいらない)
					}
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
			= LayoutInflater.from(com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.TmsUrSanmoku.this);
			View view = inflater.inflate(R.layout.setting, null);

			final EditText uid = (EditText)view.findViewById(R.id.editUID);
			final EditText fip = (EditText)view.findViewById(R.id.editFtpIP);
			final EditText fu = (EditText)view.findViewById(R.id.editFtpUsr);
			final EditText fp = (EditText)view.findViewById(R.id.editFtpPass);
			final EditText mip = (EditText)view.findViewById(R.id.editMsqlIP);
			final EditText mu = (EditText)view.findViewById(R.id.editMsqlUsr);
			final EditText mp = (EditText)view.findViewById(R.id.editMsqlPass);

			uid.setText(String.valueOf(USERID), BufferType.NORMAL);
			fip.setText(FTPIP, BufferType.NORMAL);
			fu.setText(FTPUSR, BufferType.NORMAL);
			fp.setText(FTPPASS, BufferType.NORMAL);
			mip.setText(MSQLIP, BufferType.NORMAL);
			mu.setText(MSQLUSR, BufferType.NORMAL);
			mp.setText(MSQLPASS, BufferType.NORMAL);

			Log.v(TAG,"Start!!");

			new AlertDialog.Builder(com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.TmsUrSanmoku.this)
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
							MSQLIP = mip.getText().toString();
							MSQLUSR = mu.getText().toString();
							MSQLPASS = mp.getText().toString();
							savePref("USERID","FTPIP","FTPUSR","FTPPASS","MSQLIP","MSQLUSR","MSQLPASS");
							Log.v(TAG,"positive***");
							Log.v(TAG,"USERID:" + USERID);
							Log.v(TAG,"FTPIP:" + FTPIP);
							Log.v(TAG,"FTPUSR:" + FTPUSR);
							Log.v(TAG,"FTPPASS:" + FTPPASS);
							Log.v(TAG,"MSQLIP:" + MSQLIP);
							Log.v(TAG,"MSQLUSR:" + MSQLUSR);
							Log.v(TAG,"MSQLPASS:" + MSQLPASS);
						}
					})
					.setNeutralButton("DEFAULT", new DialogInterface.OnClickListener() {          
						@Override
						public void onClick(DialogInterface dialog, int which) {
							USERID = UID;
							FTPIP = FIP;
							FTPUSR = FU;
							FTPPASS = FP;
							MSQLIP = MIP;
							MSQLUSR = MU;
							MSQLPASS = MP;
							savePref("USERID","FTPIP","FTPUSR","FTPPASS","MSQLIP","MSQLUSR","MSQLPASS");
						}
					})
					.setNegativeButton("CANCEL",null)
					.show();
			break;
		case ARInit://ARマーカをダウンロードする
			dialog = new ProgressDialog(com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.TmsUrSanmoku.this);
			dialog.setProgressStyle(ProgressDialog.STYLE_SPINNER);
			dialog.setMessage("AR Marker Initialize ...");
			dialog.show();
			//無理やり非同期処理
			AsyncTask<Void,Void,Void> ARInitTask = new AsyncTask<Void,Void,Void>(){
				@Override
				protected Void doInBackground(Void... params){
					//全ファイルのダウンロード
					@SuppressWarnings("unused")//ファイルDLしてるだけなのでunusedが出るけど実際には意味があるので無視
                        com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.InitARPatt iap = new com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.InitARPatt(FTPIP,FTPUSR,FTPPASS);
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

		case Send:
			//音声認識
			try {
				String str = "音声を入力してください";
				speechText(str);
				// インテント作成
				Intent intent = new Intent(
						RecognizerIntent.ACTION_RECOGNIZE_SPEECH); // ACTION_WEB_SEARCH
				intent.putExtra(
						RecognizerIntent.EXTRA_LANGUAGE_MODEL,
						RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
				intent.putExtra(
						RecognizerIntent.EXTRA_PROMPT,
						str);
				while(tts.isSpeaking());
				// インテント発行
				startActivityForResult(intent, REQUEST_CODE);
			} catch (ActivityNotFoundException e) {
				// このインテントに応答できるアクティビティがインストールされていない場合
				Toast.makeText(com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.TmsUrSanmoku.this,
						"ActivityNotFoundException", Toast.LENGTH_LONG).show();
			}
			break;
		case Send2:
			getDetectingObjs();
			break;

		}
	}

	//プリファレンスの取得・保存
	private void savePref(String taguid,String tagfip, String tagfu, String tagfp, String tagmip, String tagmu, String tagmp){
		SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
		pref.edit().putInt(taguid,USERID).commit();
		pref.edit().putString(tagfip,String.valueOf(FTPIP)).commit();
		pref.edit().putString(tagfu,String.valueOf(FTPUSR)).commit();
		pref.edit().putString(tagfp,String.valueOf(FTPPASS)).commit();
		pref.edit().putString(tagmip, String.valueOf(MSQLIP)).commit();
		pref.edit().putString(tagmu, String.valueOf(MSQLUSR)).commit();
		pref.edit().putString(tagmp, String.valueOf(MSQLPASS)).commit();
	}

	private void loadPref(String taguid,String tagfip, String tagfu, String tagfp, String tagmip, String tagmu, String tagmp){
		SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
		USERID = pref.getInt(taguid, UID);
		FTPIP = pref.getString(tagfip, FIP);
		FTPUSR = pref.getString(tagfu, FU);
		FTPPASS = pref.getString(tagfp, FP);
		MSQLIP = pref.getString(tagmip, MIP);
		MSQLUSR = pref.getString(tagmu, MU);
		MSQLPASS = pref.getString(tagmp, MP);
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

		//定期処理ハンドラの再開
		rutinHandler = new RutinHandler();
		rutinHandler.sleep(0);
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

		//定期処理ハンドラの停止
		rutinHandler = null;
		super.onPause();
	}

	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		Log.v("ROS", "onActivityResult");
		if (requestCode == REQUEST_CODE) {
			if(resultCode == RESULT_OK){
				String resultsString = "";
				// 結果文字列リスト
				ArrayList<String> results = data.getStringArrayListExtra(
						RecognizerIntent.EXTRA_RESULTS);

				if(results != null){
					for (String result: results) {
						//文字列が複数あった場合に結合
						resultsString += result + ",";
					}

					// トーストを使って結果を表示
					Toast.makeText(this, resultsString, Toast.LENGTH_LONG).show();
					txtInput.setText(results.get(0));
					getDetectingObjs();
				}
			}
		} else{
			super.onActivityResult(requestCode, resultCode, data);
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
		com.android.camera.Util.showFatalErrorAndFinish(com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.TmsUrSanmoku.this,
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

	private void markerResponse(){
		//何か見つかっている
		if(m_View.getFurnitureIndex()!=0){
			String name = m_View.getFurnitureName();
			Log.i("arname",name);
			int fIndex = 0;
			data.sendFurniture();
			for(TmsdbObject furniture : data.getFurnitureArray()){
				Log.i("furniturename",furniture.getName());
				if(!name.equals(furniture.getName())){
					fIndex++;
				}
				else{
					break;
				}
			}
			debugLog("name:" + name +"**index:" + fIndex);
			debugLog("numOfF:" + data.getFurnitureArray().size() + "**" + "nowid:" + data.getFurnitureArray().get(fIndex).getId());


			if(detectingObjs.size()!=0){//探索物���ある
				ArrayList<TmsdbObject> innerObjs = new ArrayList<TmsdbObject>();

				LayoutInflater inflater 
				= LayoutInflater.from(com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.TmsUrSanmoku.this);
				View view = null;

				for(TmsdbObject obj:detectingObjs){
					if(obj.getPlace()==data.getFurnitureArray().get(fIndex).getId()){
						innerObjs.add(new TmsdbObject(obj));
					}
				}

				String str = "";

				if(innerObjs.size()!=0){
					view = inflater.inflate(R.layout.systemanswer2,null);
//					ListView lv = (ListView)view.findViewById(R.id.listview2);
//
//					TmsdbObjectListAdapter adapter = new TmsdbObjectListAdapter(com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.TmsUrSanmoku.this, 0, innerObjs, savePath + saveDir);
//					lv.requestDisallowInterceptTouchEvent(true);
//					lv.setAdapter(adapter);

					TextView tv = (TextView)view.findViewById(R.id.textView8);

					str += "この中には、";
					for(TmsdbObject obj:innerObjs){
						tv.setText(obj.getName());
						str += obj.getName() + "、";
					}
					str += "が、あります";
				}
				else{
					str = "この中にお探しの物品はありません";
					view = inflater.inflate(R.layout.systemanswernoobj,null);
					TextView tv = (TextView)view.findViewById(R.id.txtSysAnswer);
					tv.setText(str);

				}

				sublayout.removeAllViews();
				sublayout.addView(view);
				speechText(str);
			}
			else{
				String str = "探索物がありません";
				Toast.makeText(com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.TmsUrSanmoku.this, str, Toast.LENGTH_LONG).show();
				speechText(str);
			}

		}

	}


	//Tag関連
	LinearLayout sublayout;//システムの回答を格納するとこ
	ArrayList<TmsdbObject> detectingObjs;
	boolean isdetecting = false;

	private void getDetectingObjs(){
		if(txtInput != null){
			dialog = new ProgressDialog(com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.TmsUrSanmoku.this);
			dialog.setProgressStyle(ProgressDialog.STYLE_SPINNER);
			dialog.setMessage("loading ...");
			dialog.show();

			AsyncTask<Void,Void,ArrayList<String>> MySqlTask = new AsyncTask<Void,Void,ArrayList<String>>(){
				@Override
				protected ArrayList<String> doInBackground(Void... params){
					ArrayList<String> results = new ArrayList<String>();
					ArrayList<String> strs = com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.tag.TagAnalyzer.divideSentence(txtInput.getText().toString());

					ArrayList<String> segs = com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.tag.TagAnalyzer.tagAnalyzer(com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.TmsUrSanmoku.this, strs);

					for(String string : segs){
						results.add(string);
						Log.v("TAG",string);
					}
					return results;
				};
				@Override
				protected void onPostExecute(ArrayList<String> result){
					dialog.dismiss();
					isdetecting = true;
					if(result.size()!=0){
						ArrayList<String> tasks = new ArrayList<String>();
						ArrayList<String> robots = new ArrayList<String>();
						ArrayList<String> objects = new ArrayList<String>();
						ArrayList<String> users = new ArrayList<String>();
						ArrayList<String> places = new ArrayList<String>();
						for(int i=0;i<result.size();i++){
							String[] split_str = result.get(i).split(":",0);
							if(split_str[0].equals("task")){
								tasks.add(split_str[1]);
							}else if(split_str[0].equals("robot")){
								robots.add(split_str[1]);
							}else if(split_str[0].equals("object")){
								objects.add(split_str[1]);
							}else if(split_str[0].equals("user")){
								users.add(split_str[1]);
							}else if(split_str[0].equals("place")){
								places.add(split_str[1]);
							}
						}

						if(tasks.size()!=0){ //命令発行
							int task_id,robot_id,object_id,user_id,place_id;

							String str;
							LayoutInflater inflater
									= LayoutInflater.from(com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.TmsUrSanmoku.this);
							View view = null;
							view = inflater.inflate(R.layout.comfirm, null);

							TextView tv = (TextView)view.findViewById(R.id.textView2);

							data.sendTag(tasks.get(0));
							task_id = data.getObjectArray().get(0).getId();
							Log.v("TASK", "task:" + data.getObjectArray().get(0).getName() + " id:" + task_id);

							str = "Task : " + data.getObjectArray().get(0).getName();
							tv.setText(str);

							tv = (TextView)view.findViewById(R.id.textView3);

							if(robots.size()!=0){
								data.sendTag(robots.get(0));
								robot_id = data.getObjectArray().get(0).getId();
								Log.v("TASK","robot:" + data.getObjectArray().get(0).getName() + " id:" + robot_id);
								str = "Robot : " + data.getObjectArray().get(0).getName();
							}else{
								robot_id = 2003;
								Log.v("TASK","robot:" + "smart_pal5_2(default)" + " id:" + robot_id);
								str = "Robot : " + "smart_pal5_2";
							}

							tv.setText(str);

							tv = (TextView)view.findViewById(R.id.textView4);

							if(places.size()!=0){
								data.sendTag(places.get(0));
								place_id = data.getObjectArray().get(0).getId();
								Log.v("TASK","place:" + data.getObjectArray().get(0).getName() + " id:" + place_id);
								str = "Place : " + data.getObjectArray().get(0).getName();
							}else{
								place_id = 0;
								Log.v("TASK","place:" + "none(default)" + " id:" + place_id);
								str = "Place : " + "none";
							}

							tv.setText(str);

							tv = (TextView)view.findViewById(R.id.textView5);

							if(users.size()!=0){
								data.sendTag(users.get(0));
								user_id = data.getObjectArray().get(0).getId();
								Log.v("TASK","user:" + data.getObjectArray().get(0).getName() + " id:" + user_id);
								str = "User : " + data.getObjectArray().get(0).getName();
							}else{
								user_id = 1002;
								Log.v("TASK","user:" + "YOU(default)" + " id:" + user_id);
								str = "User : " + "person_2_moverio";
							}

							tv.setText(str);

							tv = (TextView)view.findViewById(R.id.textView6);

							if(objects.size()!=0){
								data.sendTag(objects.get(0));
								ArrayList<TmsdbObject>objs = new ArrayList<TmsdbObject>();
								if(data.getObjectArray() != null){
									for(TmsdbObject object : data.getObjectArray()){
										boolean flag = true;
										for(int i=1; i<objects.size(); i++){
											if(object.getTag().indexOf(objects.get(i))==-1) flag = false;
										}
										if(flag == true) objs.add(object);
									}
								}
								object_id = objs.get(0).getId();
								Log.v("TASK","object:" + data.getObjectArray().get(0).getName() + " id:" + object_id);
								str = "Object : " + data.getObjectArray().get(0).getName();
							}else{
								object_id = 0;
								Log.v("TASK","object:" + "none(default)" + " id:" + object_id);
								str = "Task : " + data.getObjectArray().get(0).getName();
							}

							tv.setText(str);

							str = "以下の命令を実行します";
							speechText(str);

							data.sendCommand(task_id,robot_id,user_id,place_id,object_id);

							sublayout.removeAllViews();
							sublayout.addView(view);
						}else{ //物品検索
							if(objects.size()!=0){
								Log.i("DB", objects.get(0));

								//defaultコレクションからのデータ取得
								data.sendTag(objects.get(0));
								ArrayList<TmsdbObject> objs = new ArrayList<TmsdbObject>();
								if(data.getObjectArray() != null){
									for(TmsdbObject object : data.getObjectArray()){
										boolean flag = true;
										for(int i=1; i<objects.size(); i++){
											if(object.getTag().indexOf(objects.get(i))==-1) flag = false;
										}
										if(flag == true) objs.add(object);
									}
								}
								for(int i=0;i<objs.size();i++) {
									Log.i("DB2", objs.get(i).getName() + ":" + objs.get(i).getPlace());
								}

								//nowコレクションからのデータ取得
								detectingObjs = null;
								detectingObjs = new ArrayList<TmsdbObject>();
								for(TmsdbObject object:objs){
									data.sendInfo(object);
									if(data.getObject() != null) detectingObjs.add(new TmsdbObject(data.getObject()));
									else Log.i("DB","null");
								}

								Log.i("DB","size="+detectingObjs.size());

								if(detectingObjs.size()!=0){
									for(int i=0;i<detectingObjs.size();i++){
										Log.i("DB3",detectingObjs.get(i).getName() + ":" + detectingObjs.get(i).getPlace());
									}
								}

								//detectingObjsのplaceで家具を取得
								objs.clear();
								for(TmsdbObject object: detectingObjs){
									Log.i("DBdata",object.getName()+object.getId());
									if(object.getPlace()!=0 && object.getState()!=0){
										TmsdbObject to = new TmsdbObject();
										to.setId(object.getPlace()+100000);
										data.sendInfo(to);
										Log.i("DBdata2",data.getObject().getName());

										Log.i("DB","objsSize:" + objs.size());
										if(objs.size()==0) objs.add(new TmsdbObject(data.getObject()));
										else{
											boolean flag = true;

											for(TmsdbObject obj:objs){
												if(data.getObject().getId() == obj.getId()) flag = false;
											}

											if(flag) objs.add(new TmsdbObject(data.getObject()));
										}
									}
								}

								Log.i("DB","objsSize2:" + objs.size());
								//objsに家具情報格納完了
								String str = "";
								LayoutInflater inflater
										= LayoutInflater.from(com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.TmsUrSanmoku.this);
								View view = null;
								if(objs.size()==0){
									view = inflater.inflate(R.layout.systemanswernoobj,null);
									TextView tv = (TextView)view.findViewById(R.id.txtSysAnswer);
									str = "お探しの物品は現在切らしています";
									tv.setText(str);
								}
								else{
									view = inflater.inflate(R.layout.systemanswer,null);
									TextView tv = (TextView)view.findViewById(R.id.textView7);
//									ListView lv = (ListView)view.findViewById(R.id.listview);
//
//									TmsdbObjectListAdapter adapter = new TmsdbObjectListAdapter(com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.TmsUrSanmoku.this, 0, objs, savePath + saveDir);
//									lv.setAdapter(adapter);

									str += "お探しの物品は、";

									for(TmsdbObject obj:objs){
										Log.i("db",obj.getName());
										tv.setText(obj.getName());
										str += obj.getName() + "、";
									}

									str += "に、あります";

								}

								sublayout.removeAllViews();
								sublayout.addView(view);
								speechText(str);
								isdetecting = false;
							}
						}
					}
				}
			};

			MySqlTask.execute();
		}
	}


	//定期処理ハンドラ
	private RutinHandler rutinHandler;
	
	private class RutinHandler extends Handler {
		@Override
		public void handleMessage(Message msg) {
			//処理記述
			markerResponse();
			if (rutinHandler!=null) rutinHandler.sleep(100);
		}

		//スリープメソッド
		public void sleep(long delayMills) {
			//使用済みメッセージの削除
			removeMessages(0);
			sendMessageDelayed(obtainMessage(0),delayMills);
		}
	}

	//TTS関連
	@Override
	public void onInit(int status) {
		if (TextToSpeech.SUCCESS == status) {
			Locale locale = Locale.ENGLISH;
			if (tts.isLanguageAvailable(locale) >= TextToSpeech.LANG_AVAILABLE) {
				tts.setLanguage(locale);
			} else {
				Log.d("", "Error SetLocale");
			}
		} else {
			Log.d("", "Error Init");
		}
	}

	//読み上げ中に次のspeechTextが呼ばれた場合は無視する
	private void speechText(String string) {
		if (0 < string.length()) {
			if(!tts.isSpeaking()){
				tts.speak(string, TextToSpeech.QUEUE_FLUSH, null);
			}
			//			if (tts.isSpeaking()) {
			//				// 読み上げ中なら止める
			//				tts.stop();
			//			}
			//
			//			// 読み上げ開始
			//			tts.speak(string, TextToSpeech.QUEUE_FLUSH, null);
		}
	}


}






















