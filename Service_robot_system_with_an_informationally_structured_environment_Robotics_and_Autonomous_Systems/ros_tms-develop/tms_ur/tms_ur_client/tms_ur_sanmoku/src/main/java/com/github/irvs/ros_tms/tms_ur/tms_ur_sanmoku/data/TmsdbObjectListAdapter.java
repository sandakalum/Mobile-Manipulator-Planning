package com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.data;

import android.annotation.SuppressLint;
import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.ImageView;
import android.widget.TextView;

import com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity.R;

import java.util.List;

@SuppressLint("InflateParams")//警告を無視
public class TmsdbObjectListAdapter extends ArrayAdapter<TmsdbObject> {

	LayoutInflater layoutInflater;
	String imgpath;
	
	//コンストラクタ
	public TmsdbObjectListAdapter(Context context, int textViewResourceId,
			List<TmsdbObject> objects,String imgPath) {
		super(context, textViewResourceId, objects);
		
		layoutInflater = (LayoutInflater)context.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
		imgpath = new String(imgPath);
	}
	
	//入力の行(position)のデータ取得
	@Override
	public View getView(int position, View convertView, ViewGroup parent){
		TmsdbObject item = (TmsdbObject)getItem(position);
		
		if(convertView == null){
			convertView = layoutInflater.inflate(R.layout.listitem, null);
		}

		item.setImage(imgpath , convertView.getContext());
		
		ImageView img = null;
		img = (ImageView)convertView.findViewById(R.id.objicon);
		img.setImageBitmap(item.getImage());
		
		TextView tv = null;
		tv = (TextView)convertView.findViewById(R.id.objname);
		tv.setText(item.getName());
		
		return convertView;
	}
	
	

}
