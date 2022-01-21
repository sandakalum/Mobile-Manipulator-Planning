package com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.urclientactivity;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PixelFormat;
import android.graphics.PorterDuff;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.ImageView;

import java.util.ArrayList;

public class MyView2 extends SurfaceView implements SurfaceHolder.Callback, OnTouchListener{

	//現在認識している家具のID
	//あまり良い実装ではないがこれでIDをメインノードに伝達する
	//伝達用メソッド
	private int furnitureIndex;
	
	public int getFurnitureIndex(){
		return furnitureIndex;
	}
	
	
	private final int[] color = {
		Color.RED,
		Color.BLUE,
		Color.GREEN,
		Color.YELLOW,
		Color.BLACK,
		Color.WHITE,
		Color.GRAY,
		Color.DKGRAY
	};
	private boolean drawable = false;
	private float[] x ,y ,z;
	private float[] px,py,pz;
	private int found_markers = 0;
	private int[] ar_code_index;
	private float[][] resultf;
	private float[][] mat;
	private float[] cameraRHF;
	private float r;
	private ImageView img;
	private Bitmap on,off;
	
	private ArrayList<String> names;
	
	public void setNames(String[] names){
		for(String n : names){
			this.names.add(n);
		}
	}
	
	public String getFurnitureName(){
		return names.get(furnitureIndex);
	}

	//ポップアップ用フラグ
	private boolean touched;
	
	public boolean isTouched(){
		return touched;
	}
	
	public void clearTouched(){
		touched = false;
		this.furnitureIndex = 0;
		this.found_markers = 0;
	}
	
	
	
//	public void setFurnitures(ArrayList<TmsdbObject> furnitures){
//		this.furnitures = null;
//		this.furnitures = new ArrayList<TmsdbObject>();
//		for(int i=0;i<furnitures.size();i++){
//			this.furnitures.add(new TmsdbObject(furnitures.get(i)));
//		}
//	}
	
	public MyView2(Context context) {
		super(context);
		setWillNotDraw(false);
//		setZOrderOnTop(true);//最前列に
//		setBackgroundColor(Color.TRANSPARENT);//背景を透明に
		getHolder().setFormat(PixelFormat.TRANSLUCENT);//透過する
		getHolder().addCallback(this);
		px = py = pz = new float[8];
		this.ar_code_index = new int[8];
		this.resultf = new float[8][16];
		this.cameraRHF = new float[16];
		x = new float[8];
		y = new float[8];
		z = new float[8];
		r = 50f;
		setOnTouchListener(this);
		furnitureIndex = 0;
		names = new ArrayList<String>();
		touched = false;
//		img = (ImageView)findViewById(R.id.imageView1);
	}

	public void setParam(int found_markers, int[] ar_code_index, float[][] resultf, float[] cameraRHF){
		drawable = false;
		this.found_markers = found_markers;

		System.arraycopy(ar_code_index, 0, this.ar_code_index, 0, found_markers);
		
		//set_furnitures_id
		if(found_markers != 0) furnitureIndex = ar_code_index[0];
		else furnitureIndex = 0;
		
		for(int i=0;i<found_markers;i++){

			System.arraycopy(resultf[i], 0, this.resultf[i], 0, 16);

			this.ar_code_index[i] = ar_code_index[i];

			//x,y,z の　位置計算 x,y,zは-10～10
			x[i] = cameraRHF[0] * resultf[i][12] + cameraRHF[4] * resultf[i][13] + cameraRHF[8] * resultf[i][14] + cameraRHF[12] * resultf[i][15];
			y[i] = cameraRHF[1] * resultf[i][12] + cameraRHF[5] * resultf[i][13] + cameraRHF[9] * resultf[i][14] + cameraRHF[13] * resultf[i][15];
			z[i] = cameraRHF[2] * resultf[i][12] + cameraRHF[6] * resultf[i][13] + cameraRHF[10] * resultf[i][14] + cameraRHF[14] * resultf[i][15];
			
			//0～1.0に変換
			x[i] = (x[i] + 10f)/20f;
			y[i] = (y[i] + 10f)/20f;
			z[i] = (z[i] + 10f)/20f;
			
			//急激に変化しないようフィルタリング
			filter(x[i],px[i]);
			filter(y[i],py[i]);
			filter(z[i],pz[i]);
//			x[i] = 0.2f*x[i] + 0.8f*px[i];
//			y[i] = 0.2f*y[i] + 0.8f*py[i];
//			z[i] = 0.2f*z[i] + 0.8f*pz[i];
			
			//px,py,pzに値を代入
			px[i] = x[i];
			py[i] = y[i];
			pz[i] = z[i];

		}
		
		if(names.size()!=0) drawable = true;
		//		Log.v("MyView2","setCircleParam");

		invalidate();
	}

	public void clear(){
		drawable = false;

		invalidate();
	}

	public void filter(float val, float pre_val){
		if(!(pre_val<0.55&&pre_val>0.45)){
			if(val<0.55&&val>0.45) val = pre_val;
		}
	}
	//mat = cameraRHF*resultf[i]
	/*private void calcMat(){
		this.mat = new float[this.found_markers][16];
		for(int i=0;i<this.found_markers;i++){
			for(int j=0;j<4;j++){

				mat[i][j] = this.cameraRHF[0]*this.resultf[i][j] + this.cameraRHF[1]*this.resultf[i][j+4] + this.cameraRHF[2]*this.resultf[i][j+8] + this.cameraRHF[3]*this.resultf[i][j+11];
				mat[i][j+4] = this.cameraRHF[4]*this.resultf[i][j] + this.cameraRHF[5]*this.resultf[i][j+4] + this.cameraRHF[6]*this.resultf[i][j+8] + this.cameraRHF[7]*this.resultf[i][j+11];
				mat[i][j+8] = this.cameraRHF[8]*this.resultf[i][j] + this.cameraRHF[9]*this.resultf[i][j+4] + this.cameraRHF[10]*this.resultf[i][j+8] + this.cameraRHF[11]*this.resultf[i][j+11];
				mat[i][j+11] = this.cameraRHF[12]*this.resultf[i][j] + this.cameraRHF[13]*this.resultf[i][j+4] + this.cameraRHF[14]*this.resultf[i][j+8] + this.cameraRHF[15]*this.resultf[i][j+11];

			}
		}


	}*/

	@Override
	protected void onDraw(Canvas canvas){
		//canvas = getHolder().lockCanvas();
		if(drawable){
			Paint paint = new Paint();
			paint.setAntiAlias(true);
			paint.setStyle(Paint.Style.STROKE);
			paint.setStrokeWidth(5);
			paint.setTextSize(80);
//			
//			for(int i=0;i<found_markers;i++){
//				paint.setColor(color[ar_code_index[i]]);
//				canvas.drawText(names.get(ar_code_index[i]), x[i]*canvas.getWidth()-100, (1-y[i])*canvas.getHeight(), paint);
//			}
			
			if(found_markers>1){
				paint.setColor(Color.GREEN);
				canvas.drawText("Found Markers!",50, 450, paint);
//				img.setImageResource(R.drawable.icon_on);
			}
			
			
		} else{
			canvas.drawColor(0, PorterDuff.Mode.CLEAR);
		}
		//getHolder().unlockCanvasAndPost(canvas);
	}

	@Override
	public void surfaceChanged(SurfaceHolder arg0, int arg1, int arg2, int arg3) {
		Log.v("MyView2","surfaceChanged");
	}

	@Override
	public void surfaceCreated(SurfaceHolder arg0) {
		Log.v("MyView2","surfaceCreated");

		//		Canvas canvas = arg0.lockCanvas();
		////		canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
		//		Paint paint = new Paint();
		//		paint.setAntiAlias(true);
		//		paint.setStyle(Paint.Style.STROKE);
		//		paint.setStrokeWidth(10);
		//		paint.setColor(Color.RED);
		//		canvas.drawCircle(canvas.getWidth()/2, canvas.getHeight()/2, 50, paint);
		//		arg0.unlockCanvasAndPost(canvas);

	}

	@Override
	public void surfaceDestroyed(SurfaceHolder arg0) {
		Log.v("MyView2","surfaceDestroyed");

	}
	
	@Override
	public boolean onTouch(View v, MotionEvent event){
		
		//タッチ座標の取得
		
		float tx = event.getX();
		float ty = event.getY();
		
		touched = true;
//		if(Float.valueOf(r).equals(50f))
//			r = 100f;
//		else r = 50f;
//		invalidate();
		return true;
	}


}
