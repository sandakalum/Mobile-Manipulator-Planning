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

package com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.urclientactivity;

import android.media.MediaPlayer;
import android.os.SystemClock;
import android.util.Log;

import java.io.InputStream;
import java.util.ArrayList;

import jp.nyatla.nyartoolkit.NyARException;
import jp.nyatla.nyartoolkit.core.NyARCode;
import jp.nyatla.nyartoolkit.core.param.NyARParam;
import jp.nyatla.nyartoolkit.core.raster.rgb.NyARRgbRaster_RGB;
import jp.nyatla.nyartoolkit.core.transmat.NyARTransMatResult;
import jp.nyatla.nyartoolkit.core.types.NyARBufferType;
import jp.nyatla.nyartoolkit.core.types.matrix.NyARDoubleMatrix44;
import jp.nyatla.nyartoolkit.detector.NyARDetectMarker;
import jp.nyatla.nyartoolkit.jogl.utils.NyARGLUtil;

/**
 * ARToolKit Drawer
 *  マーカー認識部分などが含まれたクラス
 */
public class ARToolkitDrawer
{
	/**
	 * マーカーのパターン数
	 */
	private int mNumPatt;

	/**
	 * 一回の処理で検出できる最大のマーカー数。誤認識分も含まれるので少なすぎても駄目、多すぎると処理負荷増
	 */
	private static final int MARKER_MAX = 8;

	/**
	 * @see jp.nyatla.nyartoolkit.detector.NyARDetectMarker
	 */
	private NyARDetectMarker nya = null;

	/**
	 * @see jp.nyatla.nyartoolkit.core.raster.rgb.NyARRgbRaster_RGB
	 */
	private NyARRgbRaster_RGB raster = null;

	/**
	 * カメラパラメータを保持するクラス
	 * @see jp.nyatla.nyartoolkit.core.param.NyARParam
	 */
	private NyARParam ar_param = null;

	/**
	 * マーカーのパターン情報を管理するクラス
	 * @see jp.nyatla.nyartoolkit.core.param.NyARParam
	 */
	private NyARCode[] ar_code;

	/**
	 * マーカーの幅
	 */
	private double marker_width[];

	/**
	 * 検出したマーカーの座標変換行列
	 * @see jp.nyatla.nyartoolkit.core.transmat.NyARTransMatResult
	 */
	private NyARTransMatResult ar_transmat_result = new NyARTransMatResult();


	// Renderer for metasequoia model
//	private ModelRenderer mRenderer = null;
	// Renderer of min3d
//    private Renderer mRenderer;
	private MyView2 m_View;
//	private CircleView CView;

	private MediaPlayer mMediaPlayer = null;

	static {
		System.loadLibrary("yuv420sp2rgb");
	}
	public static native void decodeYUV420SP(int[] rgb, byte[] yuv420sp, int width, int height, int type);
	public static native void decodeYUV420SP(byte[] rgb, byte[] yuv420sp, int width, int height, int type);

	/**
	 * Constructor
	 *
	 * @param camePara
	 * @param patt
	 * @param mRenderer
	 * @param mTranslucentBackground
	 * @param isYuv420spPreviewFormat
	 */
	// Renderer for metasequoia model
//	public ARToolkitDrawer(InputStream camePara, int[] width, ArrayList<InputStream> patt, ModelRenderer mRenderer) {
	// Renderer of min3d
	public ARToolkitDrawer(InputStream camePara, int[] width, ArrayList<InputStream> patt, MyView2 m_View/*, CircleView CView*//*Renderer mRenderer*/) {
//		this.mRenderer = mRenderer;
		this.m_View = m_View;
//		this.CView = CView;

		this.initialization(camePara, width, patt);
	}

	/**
	 * 初期化処理
	 * マーカーパターン、カメラパラメータはここで読み込む
	 */
	private void initialization(InputStream camePara, int[] width, ArrayList<InputStream> patt) {
		mNumPatt = patt.size();
		marker_width = new double[mNumPatt];
		ar_code = new NyARCode[mNumPatt];
		try {
			for (int i = 0; i < mNumPatt; i++) {
				// マーカーの幅
				marker_width[i] = width[i];

				// マーカーの分割数
				ar_code[i] = new NyARCode(16, 16);
				ar_code[i].loadARPatt(patt.get(i));
			}

			ar_param = new NyARParam();
			ar_param.loadARParam(camePara);
		} catch (Exception e) {
			Log.e("nyar", "resource loading failed", e);
		}
	}

	/**
	 * NyARToolKitの初期化
	 *  カメラパラメータの読み込み
	 *
	 * @param w スクリーンサイズ(width)
	 * @param h スクリーンサイズ(height)
	 */
	private void createNyARTool(int w, int h) {
		// NyARToolkit setting.
		try {
			if (nya == null) {
				ar_param.changeScreenSize(w, h);

				// マーカーの枠線幅を変えることは可能。
				// NyARDetectMarker 内にコメントを追加したのでその部分を参照のこと
				nya = new NyARDetectMarker(ar_param, ar_code, marker_width, mNumPatt, NyARBufferType.BYTE1D_B8G8R8_24);
				nya.setContinueMode(true);
			}
			Log.d("nyar", "resources have been loaded");
		} catch (Exception e) {
			Log.e("nyar", "resource loading failed", e);
		}

	}

	/**
	 *
	 * @param mediaplayer
	 */
	public void setMediaPlayer(MediaPlayer mediaplayer) {
		this.mMediaPlayer = mediaplayer;
	}

	/**
	 * NyARGLUtil.toCameraFrustumRH()の出力double[]をfloat[]に変換する
	 * @param i_arparam
	 * @param o_gl_projection
	 */
	public static void toCameraFrustumRHf(NyARParam i_arparam, float[] o_gl_projection)
	{
		double[] mf = new double[16];
		NyARGLUtil.toCameraFrustumRH(i_arparam, NyARGLUtil.SCALE_FACTOR_toCameraFrustumRH_NYAR2, 10, 10000, mf);

		for (int i = 0; i < mf.length; i++) {
			o_gl_projection[i] = (float) mf[i];
		}
	}

	/**
	 * NyARGLUtil.toCameraViewRH()の出力double[]をfloat[]に変換する
	 * @param i_ny_result
	 * @param o_gl_result
	 */
	public static void toCameraViewRHf(NyARDoubleMatrix44 i_ny_result, float[] o_gl_result)
	{
		double[] mf = new double[16];
		NyARGLUtil.toCameraViewRH(i_ny_result, NyARGLUtil.SCALE_FACTOR_toCameraViewRH_NYAR2, mf);

		for (int i = 0; i < mf.length; i++) {
			o_gl_result[i] = (float) mf[i];
		}
	}

	/**
	 * 描画処理部分
	 *  メインループ処理と読み替えても良い
	 * @param data
	 */
	public void draw(byte[] data) {

		if(data == null) {
			Log.d("AR draw", "data= null");
			return;
		}
		Log.d("AR draw", "data.length= " + data.length);
		int width = 320;
		int height = 240;

		// start coordinates calculation.
		byte[] buf = new byte[width * height * 3];

		// assume YUV420SP
		long time1 = SystemClock.uptimeMillis();
		// convert YUV420SP to RGB24
		decodeYUV420SP(buf, data, width, height, 1);
		long time2 = SystemClock.uptimeMillis();
		Log.d("ARToolkitDrawer", "RGB decode time: " + (time2 - time1) + "ms");

		float[][] resultf = new float[MARKER_MAX][16];

		int found_markers;
		int ar_code_index[] = new int[MARKER_MAX];

		createNyARTool(width, height);
		// Marker detection
		try {
			Log.d("AR draw", "Marker detection.");
			raster = new NyARRgbRaster_RGB(width, height);
			raster.wrapBuffer(buf);
			found_markers = nya.detectMarkerLite(raster, 100);
		} catch (NyARException e) {
			Log.e("AR draw", "marker detection failed", e);
			return;
		}

		boolean isDetect = false;
		
		// An OpenGL object will be drawn if matched.
		if (found_markers > 0) {
			Log.d("AR draw", "!!!!!!!!!!!exist marker." + found_markers + "!!!!!!!!!!!");
			// Projection transformation.
			float[] cameraRHf = new float[16];
			toCameraFrustumRHf(ar_param, cameraRHf);

			if (found_markers > MARKER_MAX)
				found_markers = MARKER_MAX;
			for (int i = 0; i < found_markers; i++) {
				if (nya.getConfidence(i) < 0.60f)
					continue;
				try {
					ar_code_index[i] = nya.getARCodeIndex(i);
					NyARTransMatResult transmat_result = ar_transmat_result;
					nya.getTransmationMatrix(i, transmat_result);
					toCameraViewRHf(transmat_result, resultf[i]);
					isDetect = true;
//					m_View.display(found_markers,nya.getARCodeIndex(i));
				} catch (NyARException e) {
					Log.e("AR draw", "getCameraViewRH failed", e);
					return;
				}
			}
//			String str = new String("ar_code_index:");
//			for(int i=0;i<MARKER_MAX;i++) str += ar_code_index[i] + ",";
//			
//			for(int i=0;i<3;i++) Log.v("AR draw","*****");
//			Log.v("AR draw","found_markers:" + found_markers);
//			Log.v("AR draw",str);
//			for(int i=0;i<3;i++) Log.v("AR draw","*****");
			
//			mRenderer.objectPointChanged(found_markers, ar_code_index, resultf, cameraRHf);
			m_View.setParam(found_markers, ar_code_index, resultf, cameraRHf);
		} else {
			Log.d("AR draw", "not exist marker.");
			m_View.clear();
//			mRenderer.objectClear();
		}
		if (isDetect) {
			if (mMediaPlayer != null) {
				if (!mMediaPlayer.isPlaying())
					mMediaPlayer.start();
			}
		}
		else {
			if (mMediaPlayer != null) {
				if (mMediaPlayer.isPlaying())
					mMediaPlayer.pause();
			}
		}
	}
}
