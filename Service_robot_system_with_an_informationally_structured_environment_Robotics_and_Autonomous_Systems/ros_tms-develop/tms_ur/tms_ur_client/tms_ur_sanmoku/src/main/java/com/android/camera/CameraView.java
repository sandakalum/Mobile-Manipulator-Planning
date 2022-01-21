package com.android.camera;

import android.content.Context;
import android.hardware.Camera;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

public class CameraView extends SurfaceView implements SurfaceHolder.Callback{
	private Camera m_Camera;
	private SurfaceHolder m_Holder; 
	public CameraView(Context context) {
		super(context);
		m_Holder= getHolder();
		m_Holder.addCallback(this);
	}

	@Override
	public void surfaceChanged(SurfaceHolder arg0, int arg1, int arg2, int arg3) {
		if (m_Camera != null) {
			Camera.Parameters parameters = m_Camera.getParameters();
			parameters.setPreviewSize(parameters.getSupportedPreviewSizes().get(0).width,
					parameters.getSupportedPreviewSizes().get(0).height);
			m_Camera.setParameters(parameters);
//			m_Camera.setDisplayOrientation(90);
			m_Camera.startPreview();
		}
	}

	@Override
	public void surfaceCreated(SurfaceHolder arg0) {
		// カメラを起動する
		m_Camera = Camera.open();
		if(m_Camera == null){
			m_Camera = Camera.open();
		}
		try {
			m_Camera.setPreviewDisplay(m_Holder);
		} catch (Exception e) {
			e.printStackTrace();
		}

	}

	@Override
	public void surfaceDestroyed(SurfaceHolder arg0) {
		// カメラを停止する
		if (m_Camera != null) {
			m_Camera.stopPreview();
			m_Camera.release();
			m_Camera = null;
		}
	}

}