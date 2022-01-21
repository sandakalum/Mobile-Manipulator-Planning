package com.github.irvs.ros_tms.tms_ur.tms_ur_dbbrowser.ftp;

import android.util.Log;

import org.apache.commons.net.ftp.FTP;
import org.apache.commons.net.ftp.FTPClient;
import org.apache.commons.net.ftp.FTPFile;
import org.apache.commons.net.ftp.FTPReply;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.SocketException;

public class FtpClient{
	
	private FTPClient client;
	private final String cname = new String("FtpClient");
	private final int port = 21;
	private final int timeOut = 5000;
	
	private boolean isConnected;
	private String filepath;
	
	//繋がってるかどうか
	public boolean IsConnected() {
		return isConnected;
	}
	
	//インスタンス生成時コンストラクタで接続まで行う
	public FtpClient(String IP,String UID,String PASS){
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
			
			filepath = null;
			
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
	
	public void ChangeDir(String dname){
		try {
			client.changeWorkingDirectory(dname);
			Log.v(cname,client.printWorkingDirectory());
		} catch (IOException e) {
			Log.e(cname,"ChangeDirERROR "+dname,e);
		}
	}
	
	public void SetDLPass(String path,String fname){
		filepath = new String(path + fname + "/");
		File file = new File(filepath);
		if(!file.exists()) file.mkdirs();
		Log.v("SetDLPath",String.valueOf(file.exists()));
	}
	
	//imagesの中身を消す
	public void Refresh(){
		if(filepath != null){
			File file = new File(filepath);
			if(file.isDirectory()){
				String[] child = file.list();
				for(int i=0;i<child.length;i++){
					File childFile = new File(filepath,child[i]);
					childFile.delete();
				}
			}
			file.delete();
			file.mkdir();
		}
	}
	
	//ファイル名を指定してDL
	public void DownLoad(String fname){
		try {
			FileOutputStream fos = new FileOutputStream(filepath+"/"+fname);
			client.retrieveFile(fname, fos);
			fos.close();
		} catch (FileNotFoundException e) {
			Log.e(cname,fname + " is NotFound::",e);
		} catch (IOException e) {
			Log.e(cname,"[PASS]" + filepath + " is Exists?",e);
		}
	}
	
	//imagesフォルダ直下のデータをすべてDL
	public void DownloadAll(){
		try {
			FTPFile[] arrFiles = client.listFiles();
			int numOfFiles = arrFiles.length;
			for(int i=0;i<numOfFiles;i++){
				FileOutputStream fos = new FileOutputStream(filepath+arrFiles[i].getName());
				client.retrieveFile(arrFiles[i].getName(),fos);
				fos.close();
			}
		} catch (FileNotFoundException e) {
			Log.e(cname,"FileNotFoundException",e);
		} catch (IOException e) {
			Log.e(cname,"[PASS]" + filepath + " is Exists?",e);
		}
	}
	
	//解放
	public void close(){
		if(isConnected){
			client = null;
			filepath = null;
			isConnected = false;
		}
	}
}

