package com.example.rts.tms_ur_watch;

import android.content.ActivityNotFoundException;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.media.AudioAttributes;
import android.media.AudioManager;
import android.media.SoundPool;
import android.os.Bundle;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.support.wearable.activity.WearableActivity;
import android.support.wearable.view.BoxInsetLayout;
import android.util.Log;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Created by rts on 10/24/17.
 */

public class Watch extends WearableActivity implements SensorEventListener,View.OnClickListener{
    private SensorManager mSensorManager;
    private TextView textView;
    public float hb = 100.0f;
    private rosNode rosnode;
    private Sensor mStepDetectorSensor;
    private Sensor mStepCounterSensor;
    private TextView text_steps;

    private BoxInsetLayout mContainerView;
    private TextView mBatteryView;

    private AudioAttributes audioAttributes;
    private SoundPool soundPool;
    private int sound;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        setAmbientEnabled();

        rosnode = (rosNode) this.getApplication();

        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        Sensor sensor = mSensorManager.getDefaultSensor(Sensor.TYPE_HEART_RATE);
        mSensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_NORMAL);
        textView = (TextView) findViewById(R.id.hbr);
        mBatteryView = (TextView) findViewById(R.id.battery);

        mStepDetectorSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_STEP_DETECTOR);
        mStepCounterSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_STEP_COUNTER);
        mSensorManager.registerListener(this,mStepCounterSensor,SensorManager.SENSOR_DELAY_FASTEST);
        mSensorManager.registerListener(this,mStepDetectorSensor,SensorManager.SENSOR_DELAY_FASTEST);
        text_steps = (TextView) findViewById(R.id.steps);

        textView.setText("測定中");
        findViewById(R.id.VoiceButton).setOnClickListener(this);

        IntentFilter filter = new IntentFilter();
        filter.addAction(Intent.ACTION_BATTERY_CHANGED);
        registerReceiver(myReceiver,filter);

        audioAttributes = new AudioAttributes.Builder().setUsage(AudioAttributes.USAGE_GAME).setContentType(AudioAttributes.CONTENT_TYPE_SPEECH).build();
        soundPool = new SoundPool.Builder().setAudioAttributes(audioAttributes).setMaxStreams(1).build();
        sound = soundPool.load(this, R.raw.sound2, 1);
        soundPool.setOnLoadCompleteListener(new SoundPool.OnLoadCompleteListener() {
            @Override
            public void onLoadComplete(SoundPool soundPool, int sampleId, int status) {
                Log.d("debug","sampleId="+sampleId);
                Log.d("debug","status="+status);
            }
        });
        
        Log.d("watch","oncreate");

        mContainerView = (BoxInsetLayout) findViewById(R.id.container);
    }

    @Override
    public void onClick(View v){
        if(v!=null){
            if(v.getId() == R.id.VoiceButton){

                Log.d("watch","voicebutton pushed");
                Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
                intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
                intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, this.getPackageName());
                intent.putExtra(RecognizerIntent.EXTRA_PARTIAL_RESULTS, true);
                SpeechRecognizer recognizer = SpeechRecognizer.createSpeechRecognizer(this);
                recognizer.setRecognitionListener(new RecognitionListener() {
                    @Override
                    public void onReadyForSpeech(Bundle params) {
                        Log.d("watch","onReadyForSpeech");
                        Toast.makeText(Watch.this,"音声を入力してください",Toast.LENGTH_SHORT).show();
                    }

                    @Override
                    public void onBeginningOfSpeech() {

                    }

                    @Override
                    public void onRmsChanged(float rmsdB) {

                    }

                    @Override
                    public void onBufferReceived(byte[] buffer) {

                    }

                    @Override
                    public void onEndOfSpeech() {
                        soundPool.play(sound,0.5f,0.5f,1,0,1);
                        Log.d("watch","onEndOfSpeech");
                    }

                    @Override
                    public void onError(int error) {
                        switch (error) {
                            case SpeechRecognizer.ERROR_AUDIO:
                                // 音声データ保存失敗
                                Log.d("watch","error audio");
                                break;
                            case SpeechRecognizer.ERROR_CLIENT:
                                // Android端末内のエラー(その他)
                                Log.d("watch","error client");
                                break;
                            case SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS:
                                // 権限無し
                                Log.d("watch","error insufficient permissions");
                                break;
                            case SpeechRecognizer.ERROR_NETWORK:
                                // ネットワークエラー(その他)
                                Log.d("watch", "network error");
                                break;
                            case SpeechRecognizer.ERROR_NETWORK_TIMEOUT:
                                // ネットワークタイムアウトエラー
                                Log.d("watch", "network timeout");
                                break;
                            case SpeechRecognizer.ERROR_NO_MATCH:
                                // 音声認識結果無し
                                Log.d("watch", "error no match");
                                Toast.makeText(Watch.this, "分かりませんでした", Toast.LENGTH_SHORT).show();
                                break;
                            case SpeechRecognizer.ERROR_RECOGNIZER_BUSY:
                                // RecognitionServiceへ要求出せず
                                Log.d("watch", "error recognizer busy");
                                break;
                            case SpeechRecognizer.ERROR_SERVER:
                                // Server側からエラー通知
                                Log.d("watch", "error server");
                                break;
                            case SpeechRecognizer.ERROR_SPEECH_TIMEOUT:
                                // 音声入力無し
                                Log.d("watch", "error speech timeout");
                                Toast.makeText(Watch.this, "聞き取れませんでした", Toast.LENGTH_SHORT).show();
                                break;
                            default:
                        }
                    }

                    @Override
                    public void onResults(Bundle results) {
                        Log.d("watch","onResults");
                        ArrayList recData = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
                        String getData = recData.get(0).toString();

                        Log.d("watch",getData);
                        Toast.makeText(Watch.this, getData, Toast.LENGTH_SHORT).show();

                        rosnode.publishVoice(getData);
                    }

                    @Override
                    public void onPartialResults(Bundle partialResults) {
                    }

                    @Override
                    public void onEvent(int eventType, Bundle params) {

                    }
                });
                recognizer.startListening(new Intent());
            }
        }
    }

    @Override
    public void onEnterAmbient(Bundle ambientDetails) {
        Log.d("watch","onEnterAmbient");
        super.onEnterAmbient(ambientDetails);
        updateDisplay();
    }

    @Override
    public void onUpdateAmbient() {
        Log.d("watch","onUpdateAmbient");
        super.onUpdateAmbient();
        updateDisplay();
    }

    @Override
    public void onExitAmbient() {
        Log.d("watch","onExitAmbient");
        updateDisplay();
        super.onExitAmbient();
    }

    private void updateDisplay() {
        if (isAmbient()) {
            mContainerView.setBackgroundColor(getResources().getColor(android.R.color.black));
//            mTextView.setTextColor(getResources().getColor(android.R.color.white));
//            mClockView.setVisibility(View.VISIBLE);
//
//            mClockView.setText(AMBIENT_DATE_FORMAT.format(new Date()));
        } else {
            mContainerView.setBackground(null);
//            mTextView.setTextColor(getResources().getColor(android.R.color.black));
//            mClockView.setVisibility(View.GONE);
        }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_HEART_RATE && event.values[0] != 0) {
            hb = event.values[0];
            textView.setText("♥" + (int) hb);
            Log.d("heartrate", "" + (int) hb);
            rosnode.publishHBR((int) hb);
        }else if(event.sensor.getType() == Sensor.TYPE_STEP_COUNTER){
            int step = (int)event.values[0];
            text_steps.setText("👣" + step);
            Log.d("type_step_counter",""+step);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        Log.d("accuracy_change", "" + accuracy);
    }

    public BroadcastReceiver myReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
           if(intent.getAction().equals(Intent.ACTION_BATTERY_CHANGED)){
                int scale = intent.getIntExtra("scale",0);
                int level = intent.getIntExtra("level",0);
                int ratio = (int)(level*100/scale);
                mBatteryView.setText(ratio + "%");
            }
        }
    };
}
