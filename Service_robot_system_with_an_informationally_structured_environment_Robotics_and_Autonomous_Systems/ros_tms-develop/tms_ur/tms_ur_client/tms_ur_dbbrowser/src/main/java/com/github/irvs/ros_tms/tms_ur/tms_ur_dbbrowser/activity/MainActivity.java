package com.github.irvs.ros_tms.tms_ur.tms_ur_dbbrowser.activity;

import android.annotation.SuppressLint;
import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Message;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ListView;
import android.widget.TextView.BufferType;

import com.github.irvs.ros_tms.tms_ur.tms_ur_dbbrowser.data.TmsdbObject;
import com.github.irvs.ros_tms.tms_ur.tms_ur_dbbrowser.data.TmsdbObjectListAdapter;
import com.github.irvs.ros_tms.tms_ur.tms_ur_dbbrowser.ftp.FtpClient;
import com.github.irvs.ros_tms.tms_ur.tms_ur_dbbrowser.srv.TmsdbGetDataNode;
import com.github.irvs.ros_tms.tms_ur.tms_ur_dbbrowser.widget.Popup;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;
import java.util.ArrayList;

@SuppressLint("InflateParams")
public class MainActivity extends RosActivity implements View.OnClickListener {

	private final String TAG = "rostms_ur_dbbrowser.MainActivity";

	//Setting関連==================================
	private static int USERID;
	private static String FTPIP,FTPUSR,FTPPASS;

	private final int UID = 1;
	private final String FIP = "192.168.4.170";
	private final String FU = "rtsftp";
	private final String FP = "tmsftp";
	@SuppressLint("SdCardPath")
	private final String savePath = Environment.getExternalStorageDirectory().getPath() + "/Android/data/irvs.tms.activity/";
	private final String saveDir = "images";
	private final int Renew=0,Send=1,Setting=2,Main=3,Src=4;
	//=============================================

	//FTP
	private FtpClient fclient;
	private ProgressDialog dialog;

	private Button btnSend,btnRenew,btnSetting;
	private Button mainObj;
	private ImageButton srcObj;

	private Popup popup;

	public MainActivity() {
		super("tms_ur_dbbrowser", "tms_ur_dbbrowser", URI.create("http://192.168.4.170:11311/"));
	}

	private TmsdbGetDataNode data;

	@Override
	public void init(NodeMainExecutor nodeMainExecutor) {
		data = new TmsdbGetDataNode();
		NodeConfiguration nodeConfiguration 
		= NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(),getMasterUri());

		nodeConfiguration.setMasterUri(getMasterUri());
		nodeMainExecutor.execute(data, nodeConfiguration);
	}


	@Override
	public void onCreate(Bundle savedInstanceState){
		super.onCreate(savedInstanceState);

		//設定
		loadPref("USERID","FTPIP","FTPUSR","FTPPASS");
		setContentView(R.layout.main);

		btnRenew = (Button)findViewById(R.id.btnRenew);
		btnRenew.setOnClickListener(this);
		btnRenew.setId(Renew);
		btnSend = (Button)findViewById(R.id.btnSend);
		btnSend.setOnClickListener(this);
		btnSend.setId(Send);
		btnSetting = (Button)findViewById(R.id.btnSetting);
		btnSetting.setOnClickListener(this);
		btnSetting.setId(Setting);

		mainObj = (Button)findViewById(R.id.btnCurrentObj);
		mainObj.setOnClickListener(this);
		mainObj.setId(Main);

		srcObj = (ImageButton)findViewById(R.id.btnReturn);
		srcObj.setOnClickListener(this);
		srcObj.setId(Src);

	}

	@Override
	public void onClick(View v) {
		switch(v.getId()){
		case Renew:
			dialog = new ProgressDialog(MainActivity.this);
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

		case Send:
			TmsdbObject obj = new TmsdbObject();
			obj.setId(5002);
			data.sendInfo(obj);

			currentObject = null;
			currentObject = new TmsdbObject(data.getObject());

			drawWindow();
			break;

		case Setting:
			LayoutInflater inflater 
			= LayoutInflater.from(MainActivity.this);
			View view = inflater.inflate(R.layout.setting, null);


			final EditText uid = (EditText)view.findViewById(R.id.editUID);
			final EditText fip = (EditText)view.findViewById(R.id.editFtpIP);
			final EditText fu = (EditText)view.findViewById(R.id.editFtpUsr);
			final EditText fp = (EditText)view.findViewById(R.id.editFtpPass);

			uid.setText(String.valueOf(USERID), BufferType.NORMAL);
			fip.setText(FTPIP, BufferType.NORMAL);
			fu.setText(FTPUSR, BufferType.NORMAL);
			fp.setText(FTPPASS, BufferType.NORMAL);

			Log.v(TAG,"Start!!");

			new AlertDialog.Builder(MainActivity.this)
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

		case Main:
			if(currentObject!=null){
				popup = new Popup(this, savePath + saveDir);
				popup.draw(currentObject);
				break;
			}

		case Src:
			if(currentObject!=null){
				TmsdbObject object = new TmsdbObject();
				object.setId(currentObject.getPlace());

				data.sendInfo(object);

				currentObject = null;
				currentObject = new TmsdbObject(data.getObject());

				drawWindow();
			}
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

	private TmsdbObject currentObject;

	ListView objLayout=null;

	private void drawWindow(){
		if(currentObject != null){
			data.sendBelongObject(currentObject);
			mainObj.setText(currentObject.getName());


			if(objLayout==null) objLayout = (ListView)findViewById(R.id.listview);
			//スクロール位置の記憶
			int pos = objLayout.getFirstVisiblePosition();
			int yOff = 0;//objLayout.getChildAt(0).getTop();


			ArrayList<TmsdbObject> objs = new ArrayList<TmsdbObject>();
			for(TmsdbObject object : data.getObjectArray()){
				/*if(object.getState()==1)*/ objs.add(object);
			}

			TmsdbObjectListAdapter adapter = new TmsdbObjectListAdapter(this, 0, objs, savePath + saveDir);

			objLayout.setAdapter(adapter);
			//スクロール位置のセット
			objLayout.setSelectionFromTop(pos, yOff);

			final ArrayList<TmsdbObject> params = objs;

			objLayout.setOnItemClickListener(new OnItemClickListener(){

				@Override
				public void onItemClick(AdapterView<?> adapter, View v,
						int position, long id) {
					//					Toast.makeText(v.getContext(), "pos:"+ position + "_id:" + id, Toast.LENGTH_LONG).show();
					if(!params.get(position).getType().equals(TmsdbObject.TYPE._FURNITURE)
							&&!params.get(position).getType().equals(TmsdbObject.TYPE._SPACE)
							&&!params.get(position).getType().equals(TmsdbObject.TYPE._ROBOT)){
						popup = new Popup(MainActivity.this, savePath + saveDir);
						popup.draw(params.get(position));
					}
					else{
						currentObject = null;
						currentObject = new TmsdbObject(params.get(position));
						drawWindow();
					}
				}

			});

		}

	}

	@Override
	public void onResume(){
		super.onResume();
		rutinHandler = new RutinHandler();
		rutinHandler.sleep(0);
	}



	//定期処理ハンドラ

	private RutinHandler rutinHandler;

	private class RutinHandler extends Handler {
		@Override
		public void handleMessage(Message msg) {
			//処理記述
			drawWindow();
			if (rutinHandler!=null) rutinHandler.sleep(3000);
		}

		//スリープメソッド
		public void sleep(long delayMills) {
			//使用済みメッセージの削除
			removeMessages(0);
			sendMessageDelayed(obtainMessage(0),delayMills);
		}
	}

}
