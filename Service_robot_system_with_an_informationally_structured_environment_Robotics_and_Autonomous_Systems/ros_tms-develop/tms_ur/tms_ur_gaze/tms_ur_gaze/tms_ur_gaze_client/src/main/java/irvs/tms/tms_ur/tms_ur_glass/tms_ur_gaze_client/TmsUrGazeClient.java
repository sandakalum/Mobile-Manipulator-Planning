package irvs.tms.tms_ur.tms_ur_glass.tms_ur_gaze_client;

import android.content.Intent;
import android.media.MediaRecorder;
import android.os.Bundle;
import android.os.Environment;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.text.format.Time;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.util.ArrayList;

/**
 * tms_ur_glass_vicon client
 *
 *  - Speech recognition
 *     - Executes recognition services on worker threads, using SpeechRecognizer intent
 *     - Publishes to the topic, /recognition_text; another c++ nodes subscribes it.
 */

public class TmsUrGazeClient extends RosActivity implements View.OnClickListener,RecognitionListener,SurfaceHolder.Callback
{
    private final String TAG = "tms_ur_gaze_client/tms_ur_gaze_client";

    private SurfaceHolder v_holder;
    private MediaRecorder mediaRecorder;
    private boolean isRecording;

    private java.lang.Object mSpeechRecognizer;
    private RecognitionClient recognitionClient;

    private Intent intent;
    private TextView guiMessage;
    public static TextView guiResult;

    public TmsUrGazeClient() {
        super("tms_ur_gaze_client","tms_ur_gaze_client");
    }

    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        // SurfaceHolder
        SurfaceView surfaceView = (SurfaceView) findViewById(R.id.surface_view);
        SurfaceHolder holder = surfaceView.getHolder();
        holder.addCallback(this);
        holder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);

        Button button1 = (Button)findViewById(R.id.button);
        button1.setOnClickListener(this);

        Button button2 = (Button)findViewById(R.id.start_recoding);
        button2.setOnClickListener(this);

        Button button3 = (Button)findViewById(R.id.stop_recoding);
        button3.setOnClickListener(this);

        guiMessage = (TextView)findViewById(R.id.message);
        guiResult = (TextView) findViewById(R.id.result);

        mediaRecorder = new MediaRecorder();

        mSpeechRecognizer = SpeechRecognizer.createSpeechRecognizer(this);
        ((SpeechRecognizer) mSpeechRecognizer).setRecognitionListener(this);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration
                = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());

        recognitionClient = new RecognitionClient();
        nodeMainExecutor.execute(recognitionClient, nodeConfiguration);
    }

    @Override
    public void onClick(View v) {
        /**
         * Executes speech recognition service
         * if the button selected on UI thread.
         */
        switch (v.getId()) {
            case R.id.button:
                intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
                intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
                intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, getPackageName());
                ((SpeechRecognizer) mSpeechRecognizer).startListening(intent);
                break;
            case R.id.start_recoding:
                if (!isRecording) {
                    isRecording = true;
                    Log.d(TAG, "start recording");
                    initVideo();
                    mediaRecorder.start();
                }
                break;
            case R.id.stop_recoding:
                if (isRecording) {
                    isRecording = false;
                    Log.d(TAG, "stop recording");
                    mediaRecorder.stop();
                    mediaRecorder.reset();
                }
                break;
            default:
                break;
        }
    }

    //----------------------------------------------------------------------------------------------
    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        //
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        v_holder = holder;
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        //
    }

    //----------------------------------------------------------------------------------------------
    public void initVideo() {
        Time time = new Time("Asia/Tokyo");
        time.setToNow();
        String directory
            = String.valueOf(time.year)
            + String.valueOf(time.month + 1)
            + String.valueOf(time.monthDay)
            + String.valueOf(time.hour)
            + String.valueOf(time.minute)
            + String.valueOf(time.second);

        /**
         * Video source;  camera
         * Video format;  MPEG 4
         * Video Encoder; MPEG 4 SP
         */
        mediaRecorder.setVideoSource(MediaRecorder.VideoSource.CAMERA);
        mediaRecorder.setOutputFormat(MediaRecorder.OutputFormat.MPEG_4);
        mediaRecorder.setVideoEncoder(MediaRecorder.VideoEncoder.MPEG_4_SP);

        /**
         * Prefix;     .mp4
         * Frame rate; 30FPS
         * Frame size; 640 x 480
         */
        mediaRecorder.setVideoFrameRate(60);
        mediaRecorder.setVideoSize(640,480);
        mediaRecorder.setPreviewDisplay(v_holder.getSurface());

        mediaRecorder.setOutputFile(Environment.getExternalStorageDirectory()
            + "/tms_ur_gaze_client/" + directory + ".mp4");

        try {
            mediaRecorder.prepare(); // preparation
        } catch (Exception e) {
            Log.e(TAG, e.getMessage());
        }
    }

    //---------------------------------------------------------------------------------------------------------------
    /**
     * Overridden functions for SpeechRecognizer intent
     *
     */
    @Override
    public void onReadyForSpeech(Bundle params) {
        guiMessage.setText("onReadyForSpeech");
    }

    @Override
    public void onBeginningOfSpeech() {
        guiMessage.setText("onBeginningOfSpeech");
    }

    @Override
    public void onBufferReceived(byte[] buffer) {
        Log.v("DEBUG", "onBufferReceived");
    }

    @Override
    public void onRmsChanged(float rmsdB) {
        Log.v("DEBUG","receive : " + rmsdB + "dB");
    }

    @Override
    public void onEndOfSpeech() {
        guiMessage.setText("onEndOfSpeech");
    }

    @Override
    public void onError(int error) {
        switch (error) {
            case SpeechRecognizer.ERROR_AUDIO:
                guiMessage.setText("ERROR_AUDIO");
                break;
            case SpeechRecognizer.ERROR_CLIENT:
                guiMessage.setText("ERROR_CLIENT");
                break;
            case SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS:
                guiMessage.setText("ERROR_INSUFFICIENT_PERMISSIONS");
                break;
            case SpeechRecognizer.ERROR_NETWORK:
                guiMessage.setText("ERROR_NETWORK");
                break;
            case SpeechRecognizer.ERROR_NETWORK_TIMEOUT:
                guiMessage.setText("ERROR_NETWORK_TIMEOUT");
                break;
            case SpeechRecognizer.ERROR_NO_MATCH:
                guiMessage.setText("ERROR_NO_MATCH");
                ((SpeechRecognizer) mSpeechRecognizer).cancel();
                ((SpeechRecognizer) mSpeechRecognizer).startListening(intent);
                break;
            case SpeechRecognizer.ERROR_RECOGNIZER_BUSY:
                guiMessage.setText("ERROR_RECOGNIZER_BUSY");
                ((SpeechRecognizer) mSpeechRecognizer).cancel();
                ((SpeechRecognizer) mSpeechRecognizer).startListening(intent);
                break;
            case SpeechRecognizer.ERROR_SERVER:
                guiMessage.setText("ERROR_SERVER");
                break;
            case SpeechRecognizer.ERROR_SPEECH_TIMEOUT:
                guiMessage.setText("ERROR_SPEECH_TIMEOUT(No input)");
                ((SpeechRecognizer) mSpeechRecognizer).cancel();
                ((SpeechRecognizer) mSpeechRecognizer).startListening(intent);
                break;
            default:
        }
    }

    @Override
    public void onEvent(int eventType, Bundle params) {
        Log.v("DEBUG", "onEvent");
    }

    @Override
    public void onPartialResults(Bundle partialResults) {
        /**
         * For Vuzix M100 smart glass.
         */
        Log.v(TAG, "onPartialResults");
        guiMessage.setText("onPartialResults");

        ArrayList<String> recData = partialResults.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
        final String[] str_items = recData.toArray(new String[recData.size()]);
        recognitionClient.sendText(str_items[0]);
        ((SpeechRecognizer) mSpeechRecognizer).startListening(intent);
    }

    @Override
    public void onResults(Bundle results) {
        Log.i(TAG,"Send recognized result to server");
        ArrayList<String> recData = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
        final String[] str_items = recData.toArray(new String[recData.size()]);
        recognitionClient.sendText(str_items[0]);
        ((SpeechRecognizer) mSpeechRecognizer).startListening(intent);
    }
}