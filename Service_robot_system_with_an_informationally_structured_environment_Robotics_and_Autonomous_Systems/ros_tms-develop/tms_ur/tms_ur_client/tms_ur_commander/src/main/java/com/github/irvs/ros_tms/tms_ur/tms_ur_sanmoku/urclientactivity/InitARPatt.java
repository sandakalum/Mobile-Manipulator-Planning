//ARマーカダウンロード用クラス
//そもそもFTPまわりは実装がへんてこりんなので根本的に直す必要がある
//FTPまわりの問題点に関してはFtpClientクラスを参照

package com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity;

import android.os.Environment;
import android.util.Log;

import com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.data.TmsdbObject;

import org.apache.commons.net.ftp.FTP;
import org.apache.commons.net.ftp.FTPClient;
import org.apache.commons.net.ftp.FTPFile;
import org.apache.commons.net.ftp.FTPReply;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.SocketException;
import java.util.ArrayList;

public class InitARPatt {
	private FTPClient client;
	private final String cname = new String("FtpClient");
	private final int port = 21;
	private final int timeOut = 5000;
	private final String pattDir = new String("2D/ar_pattern");// 前に/を入れるとエラー
	private final String filepath = new String(Environment.getExternalStorageDirectory().getPath() + "/Android/data/com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity/patt/");
	private boolean isConnected;

	public InitARPatt(String IPAddress,String Usr,String Pass, ArrayList<TmsdbObject> furnitures){

		//接続
		initFTP(IPAddress,Usr,Pass);

		//ディレクトリ移動
		try {
			client.changeWorkingDirectory(pattDir);
		} catch (IOException e) {
			e.printStackTrace();
			Log.e(cname,"Change Dir:no such dir",e);
		}

		//ローカルディレクトリの初期化
		File file = new File(filepath);
		if(file.exists()){
			for(String c:file.list()){
				File childFile = new File(filepath,c);
				childFile.delete();
			}
		}
		else file.mkdirs();

		int cnt = 0;
		//入力から判別したファイルをDL
		try {
			for(cnt=0;cnt<furnitures.size();cnt++){
				FileOutputStream fos = new FileOutputStream(filepath+furnitures.get(cnt).getId() + ".pat");
				client.retrieveFile(furnitures.get(cnt).getId() + ".pat",fos);
				fos.close();
			}


		} catch (FileNotFoundException e) {
			Log.v(cname,"FileNotFoundException");
			furnitures.remove(cnt);
			--cnt;
			e.printStackTrace();	
		} catch (IOException e) {
			Log.v(cname,"IOException");
			e.printStackTrace();
		}

		if(isConnected) finalizeFTP();


	}

	
	public InitARPatt(String IPAddress,String Usr,String Pass){
		//接続
		initFTP(IPAddress,Usr,Pass);

		//ディレクトリ移動
		try {
			client.changeWorkingDirectory(pattDir);
		} catch (IOException e) {
			e.printStackTrace();
			Log.e(cname,"Change Dir:no such dir",e);
		}

		//ローカルディレクトリの初期化
		File file = new File(filepath);
		if(file.exists()){
			file.delete();
			file.mkdirs();
		}
		else file.mkdirs();

		//全ファイルをDL
		try {
			FTPFile[] files = client.listFiles();

			for(int i=0;i<files.length;i++){
				FileOutputStream fos = new FileOutputStream(filepath+files[i].getName());
				client.retrieveFile(files[i].getName(),fos);
				fos.close();
			}


		} catch (FileNotFoundException e) {
			Log.v(cname,"FileNotFoundException");
			e.printStackTrace();	
		} catch (IOException e) {
			Log.v(cname,"IOException");
			e.printStackTrace();
		}
		
		//ファイならイズ
		if(isConnected) finalizeFTP();
	}


	private void initFTP(String IP,String UID,String PASS){
		isConnected = false;
		try {
			client = new FTPClient();
			//タイムアウト設定
			client.setDefaultTimeout(timeOut);
			//接続
			client.connect(IP,port);

			if (FTPReply.isPositiveCompletion(client.getReplyCode()) == false) {
				// 接続エラー
				throw new Exception(
						new StringBuffer("FtpConnectionERROR ReplyCode=")
						.append(client.getReplyCode()).toString()
						);
			}
			//ソケットタイムアウト設定
			client.setSoTimeout(timeOut);
			if (client.login(UID,PASS) == false) {
				// 認証エラー
				throw new Exception(
						new StringBuffer("FtpPassERROR ReplyCode=")
						.append(client.getReplyCode()).toString()
						);
			}
			//接続設定
			client.setFileType(FTP.BINARY_FILE_TYPE);
			client.enterLocalPassiveMode();
			client.setDataTimeout(timeOut);

			isConnected = true;
		} catch (SocketException e) {
			Log.e(cname,"SocketException",e);
			Log.e(cname,"IP:"+IP+"port:"+port,e);
		} catch (IOException e) {
			Log.e(cname,"IOException",e);
			Log.e(cname,"IP:"+IP+"port:"+port,e);
		} catch (Exception e) {
			Log.e(cname, "UnknownException",e);
			Log.e(cname,"IP:"+IP+"port:"+port,e);
		}
	}

	private void finalizeFTP(){
		// ファイル転送先ホストからログアウトして切断する
		try {
			client.logout();
			client.disconnect();
		} catch (IOException e) {
			// ログアウトもしくは切断できなかったとき
			System.out.println("Disconnection failed!");
		}
		return;
	}

}
