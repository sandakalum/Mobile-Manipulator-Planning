package com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.widget;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.ImageView;
import android.widget.TextView;

import com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.data.TmsdbObject;
import com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity.R;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;

public class Popup{

	private
	Context con;
	String imgPath;
	Bitmap imgbitmap;
	ImageView img;
	TextView objId;
	TextView objName;
	TextView objPlace;
	TextView objWeight;
	TextView objX;
	TextView objY;
	TextView objZ;
	TextView objRoll;
	TextView objPitch;
	TextView objYaw;
	LayoutInflater inflater;
	View view;
	boolean isSetView;

	public
	Popup(Context context,String imagePath){
		con = context;
		imgPath = new String(imagePath);
		inflater = LayoutInflater.from(con);
		view = inflater.inflate(R.layout.popupdetail, null);
		img = (ImageView)view.findViewById(R.id.img);
		objId = (TextView)view.findViewById(R.id.txtId);
		objName = (TextView)view.findViewById(R.id.txtName);
		objPlace = (TextView)view.findViewById(R.id.txtPlace);
		objWeight = (TextView)view.findViewById(R.id.txtWeight);
		objX = (TextView)view.findViewById(R.id.txtX);
		objY = (TextView)view.findViewById(R.id.txtY);
		objZ = (TextView)view.findViewById(R.id.txtZ);
		objRoll = (TextView)view.findViewById(R.id.txtRoll);
		objPitch = (TextView)view.findViewById(R.id.txtPitch);
		objYaw = (TextView)view.findViewById(R.id.txtYaw);
		imgbitmap = null;
		isSetView = false;
	}

	public View getDatailView(){
		return view;
	}
	
	public void setView(TmsdbObject object){
		String imname = String.valueOf(object.getId()) + ".JPG";
		File fimage = new File(imgPath,imname);
		InputStream is;
		try {
			is = new FileInputStream(fimage);
			imgbitmap = BitmapFactory.decodeStream(is);
			is.close();
		} catch (FileNotFoundException e) {
			// TODO 自動生成された catch ブロック
			imgbitmap = BitmapFactory.decodeResource(con.getResources(), R.drawable.noimagebig);
			e.printStackTrace();
		} catch (IOException e) {
			// TODO 自動生成された catch ブロック
			e.printStackTrace();
		}

		if(imgbitmap != null) img.setImageBitmap(imgbitmap);
		//imgのセット****************************************************

		img.setImageBitmap(imgbitmap);
		objId.setText(String.valueOf(object.getId()));
		objName.setText(object.getName());
		objPlace.setText(String.valueOf(object.getPlace()));
		objWeight.setText(String.valueOf(object.getWeight()));
		objX.setText(String.valueOf(object.getX()));
		objY.setText(String.valueOf(object.getY()));
		objZ.setText(String.valueOf(object.getZ()));
		objRoll.setText(String.valueOf(object.getRr()));
		objPitch.setText(String.valueOf(object.getRp()));
		objYaw.setText(String.valueOf(object.getRy()));
		isSetView = true;
	}
	
	//詳細のポップアップを表示
	public void draw(TmsdbObject object){
		
		if(!isSetView){
			setView(object);
			isSetView = true;
		}
		
		Log.v("POPUP","Start!!");

		new AlertDialog.Builder(con)
		.setTitle("DETAIL")
		.setIcon(R.drawable.ic_launcher)
		.setView(view)
		.setPositiveButton(
				"GETOBJECT", 
				new DialogInterface.OnClickListener() {          
					@Override
					public void onClick(DialogInterface dialog, int which) {
						//ここにTSとの通信処理を記述
					}
				})
				.setNegativeButton("CANCEL",new DialogInterface.OnClickListener() {          
					@Override
					public void onClick(DialogInterface dialog, int which) {

					}})
					.show();
	}

}