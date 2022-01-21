package com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity;

import android.util.Log;

import com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.data.TmsdbObject;

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

//	private ArrayList<InputStream> patt; //pattern登録用
	
	private FTPClient client;
//	private final String IPAddress = "192.168.4.118";
//	private final String Usr = "tmsftp";
//	private final String Pass = "tmsftp";
	private final String cname = new String("FtpClient");
	private final int port = 21;
	private final int timeOut = 5000;
	private final String pattDir = new String("2D/ar_pattern");// 前に/を入れるとエラー
	private final String filepath = new String("/sdcard/Android/data/jp.androidgroup.nyartoolkit/patt/");
	private boolean isConnected;

	public InitARPatt(String IPAddress,String Usr,String Pass, ArrayList<TmsdbObject> furnitures){

//		patt = new ArrayList<InputStream>();

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

		//		//全ファイルをDL
		//		try {
		//			FTPFile[] files = client.listFiles();
		//
		//			for(int i=0;i<files.length;i++){
		//				//				Log.v(this.cname,client.printWorkingDirectory()+"/"+files[i].getName());
		//				//				patt.add(new FileInputStream(client.printWorkingDirectory()+"/"+files[i].getName()));
		//
		//				FileOutputStream fos = new FileOutputStream(filepath+files[i].getName());
		//				client.retrieveFile(files[i].getName(),fos);
		//				fos.close();
		//			}
		//
		//
		//		}
		int cnt = 0;
		//入力から判別したファイルをDL
		try {
			FTPFile[] files = client.listFiles();

			for(cnt=0;cnt<furnitures.size();cnt++){
				//				Log.v(this.cname,client.printWorkingDirectory()+"/"+files[i].getName());
				//				patt.add(new FileInputStream(client.printWorkingDirectory()+"/"+files[i].getName()));

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

//		//pattにパターンをセット
//		for(int i=0;i<file.list().length;i++){
//			try {
//				patt.add(new FileInputStream(filepath + file.listFiles()[i].getName()));
//			} catch (FileNotFoundException e) {
//				// TODO 自動生成された catch ブロック
//				e.printStackTrace();
//			}
//
//		}

		if(isConnected) finalizeFTP();


	}
//	public InitARPatt(){
//
//		patt = new ArrayList<InputStream>();
//
//		//接続
//		initFTP(IPAddress,Usr,Pass);
//
//		//ディレクトリ移動
//		try {
//			client.changeWorkingDirectory(pattDir);
//		} catch (IOException e) {
//			e.printStackTrace();
//			Log.e(cname,"Change Dir:no such dir",e);
//		}
//
//		//ローカルディレクトリの初期化
//		File file = new File(filepath);
//		if(file.exists()){
//			file.delete();
//			file.mkdirs();
//		}
//		else file.mkdirs();
//
//		//全ファイルをDL
//		try {
//			FTPFile[] files = client.listFiles();
//
//			for(int i=0;i<files.length;i++){
//				//				Log.v(this.cname,client.printWorkingDirectory()+"/"+files[i].getName());
//				//				patt.add(new FileInputStream(client.printWorkingDirectory()+"/"+files[i].getName()));
//
//				FileOutputStream fos = new FileOutputStream(filepath+files[i].getName());
//				client.retrieveFile(files[i].getName(),fos);
//				fos.close();
//			}
//
//
//		} catch (FileNotFoundException e) {
//			Log.v(cname,"FileNotFoundException");
//			e.printStackTrace();	
//		} catch (IOException e) {
//			Log.v(cname,"IOException");
//			e.printStackTrace();
//		}
//		
//		//ファイならイズ
//		if(isConnected) finalizeFTP();
//
//		//pattにパターンをセット
//		for(int i=0;i<file.list().length;i++){
//			try {
//				patt.add(new FileInputStream(filepath + file.listFiles()[i].getName()));
//			} catch (FileNotFoundException e) {
//				// TODO 自動生成された catch ブロック
//				e.printStackTrace();
//			}
//
//		}
//	}
	
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
				//				Log.v(this.cname,client.printWorkingDirectory()+"/"+files[i].getName());
				//				patt.add(new FileInputStream(client.printWorkingDirectory()+"/"+files[i].getName()));

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

//	public ArrayList<InputStream> getPatt(){
//		return patt;
//	}

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
