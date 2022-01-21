package com.github.irvs.ros_tms.tms_ur.tms_ur_dbbrowser.data;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.github.irvs.ros_tms.tms_ur.tms_ur_dbbrowser.activity.R;

import org.ros.internal.message.RawMessage;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;

import tms_msg_db.Tmsdb;

public class TmsdbObject implements Tmsdb {
	//メソッド
	private int id;
	private int place;
	private int state;
	private String name;
	//TYPE:person, robot, sensor, structure, space, furniture, object, task, subtask
	private String type;
	//位置情報(mm, deg)
	private double x,y,z,rr,rp,ry;
	//オフセット
	private double offset_x, offset_y, offset_z;
	//質量(g)
	private double weight;
	
	private String etcdata;	//温度等のデータ
	private String extfile;	//extfile path for FTP
	private String joint;	//関節角度?
	private String note;	//備考
	private String task;	//タスク
	private String time;	//時間
	private double probability;	//確率
	private String rfId;	//RFIDタグ
	private int sensor;	//センサID
	private String tag;
	
	//Bitmapimage
	private Bitmap bitmap;
	
	/************************************************
	 * コンストラクタ
	 * 引数なし, 引数Tmsdb
	 ************************************************/
	public TmsdbObject(){
		time = new String();
		type = new String();
		id = 0;
		name = new String();
		x = y = z = rr = rp = ry = 0.0;
		offset_x = offset_y = offset_z = 0.0;
		joint = new String();
		weight = 0.0;
		rfId = new String();
		etcdata = new String();
		place = 0;
		extfile = new String();
		sensor = 0;
		probability = 0.0;
		state = 0;
		task = new String();
		note = new String();
		tag = new String();
	}
	
	public TmsdbObject(Tmsdb tmsdb){
		id = tmsdb.getId();
		place = tmsdb.getPlace();
		state = tmsdb.getState();
		name = new String(tmsdb.getName());
		type = new String(tmsdb.getType());
		x = tmsdb.getX();
		y = tmsdb.getY();
		z = tmsdb.getZ();
		rp = tmsdb.getRp();
		rr = tmsdb.getRp();
		ry = tmsdb.getRy();
		offset_x = tmsdb.getOffsetX();
		offset_y = tmsdb.getOffsetY();
		offset_z = tmsdb.getOffsetZ();
		weight = tmsdb.getWeight();
		etcdata = new String(tmsdb.getEtcdata());
		extfile = new String(tmsdb.getExtfile());
		joint = new String(tmsdb.getJoint());
		note = new String(tmsdb.getNote());
		task = new String(tmsdb.getTask());
		probability = tmsdb.getProbability();
		rfId = tmsdb.getRfid();
		sensor = tmsdb.getSensor();
		tag = tmsdb.getTag();
	}
	
	
	/************************************************
	 * ここからセッター
	 ************************************************/
	//Bitmapのセット
	//アイコンは必ずしもセットする必要があるわけではないので個別に
	public void setImage(String path, Context context){
		if(bitmap == null){
			File fimage = new File(path, this.getId() + ".JPG");
			InputStream is;
			
			try {
				is = new FileInputStream(fimage);
				bitmap = BitmapFactory.decodeStream(is);
				is.close();
			} catch (FileNotFoundException e) {
				bitmap = BitmapFactory.decodeResource(context.getResources(), R.drawable.noimagemin);
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
	
	@Override
	public void setEtcdata(String arg0) {
		etcdata = arg0;
	}

	@Override
	public void setExtfile(String arg0) {
		extfile = arg0;
	}

	@Override
	public void setId(int arg0) {
		id = arg0;
	}

	@Override
	public void setJoint(String arg0) {
		joint = arg0;
	}

	@Override
	public void setName(String arg0) {
		name  = arg0;

	}

	@Override
	public void setNote(String arg0) {
		note = arg0;
	}

	@Override
	public void setOffsetX(double arg0) {
		offset_x = arg0;
	}

	@Override
	public void setOffsetY(double arg0) {
		offset_y = arg0;
	}

	@Override
	public void setOffsetZ(double arg0) {
		offset_z = arg0;
	}

	@Override
	public void setPlace(int arg0) {
		place = arg0;
	}

	@Override
	public void setProbability(double arg0) {
		probability = arg0;
	}

	@Override
	public void setRfid(String arg0) {
		rfId = arg0;
	}

	@Override
	public void setRp(double arg0) {
		rp = arg0;
	}

	@Override
	public void setRr(double arg0) {
		rr = arg0;
	}

	@Override
	public void setRy(double arg0) {
		ry = arg0;
	}

	@Override
	public void setSensor(int arg0) {
		sensor = arg0;
	}

	@Override
	public void setState(int arg0) {
		state = arg0;
	}

	@Override
	public void setTask(String arg0) {
		task = arg0;
	}

	@Override
	public void setTime(String arg0) {
		time = arg0;
	}

	//引数には定数クラスTYPEを使用する
	@Override
	public void setType(String arg0) {
		type = arg0;
	}

	@Override
	public void setWeight(double arg0) {
		 weight = arg0;
	}

	@Override
	public void setX(double arg0) {
		x = arg0;

	}

	@Override
	public void setY(double arg0) {
		y = arg0;
	}

	@Override
	public void setZ(double arg0) {
		z = arg0;
	}
	
	@Override
	public void setTag(String arg0) {
		tag = arg0;
	}

	/************************************************
	 * ここからゲッター
	 ************************************************/
	
	public Bitmap getImage(){
		return bitmap;
	}
	
	@Override
	public int getId() {
		return id;
	}
	
	@Override
	public int getPlace() {
		return place;
	}
	
	@Override
	public String getName() {
		return name;
	}
	
	@Override
	public String getType() {
		return type;
	}

	@Override
	public double getWeight() {
		return weight;
	}

	@Override
	public double getX() {
		return x;
	}

	@Override
	public double getY() {
		return y;
	}

	@Override
	public double getZ() {
		return z;
	}
	
	@Override
	public double getRp() {
		return rp;
	}

	@Override
	public double getRr() {
		return rr;
	}

	@Override
	public double getRy() {
		return ry;
	}
	
	@Override
	public double getOffsetX() {
		return offset_x;
	}

	@Override
	public double getOffsetY() {
		return offset_y;
	}

	@Override
	public double getOffsetZ() {
		return offset_z;
	}
	
	
	
	@Override
	public RawMessage toRawMessage() {
		return null;
	}

	@Override
	public String getEtcdata() {
		return etcdata;
	}

	@Override
	public String getExtfile() {
		return extfile;
	}

	@Override
	public String getJoint() {
		return joint;
	}

	@Override
	public String getNote() {
		return note;
	}

	@Override
	public double getProbability() {
		return probability;
	}

	@Override
	public String getRfid() {
		return rfId;
	}

	@Override
	public int getSensor() {
		return sensor;
	}

	@Override
	public int getState() {
		return state;
	}

	@Override
	public String getTask() {
		return task;
	}

	@Override
	public String getTime() {
		return time;
	}

	@Override
	public String getTag() {
		return tag;
	}

	//type用の定数クラス
	public class TYPE{
		private TYPE(){};
		public static final String _PERSON = "person";
		public static final String _ROBOT = "robot";
		public static final String _SENSOR = "sensor";
		public static final String _STRUCTURE = "structure";
		public static final String _SPACE = "space";
		public static final String _FURNITURE = "furniture";
		public static final String _OBJECT = "object";
		public static final String _TASK = "task";
		public static final String _SUBTASK = "subtask";
		public static final String _STATE = "state";
	}


}