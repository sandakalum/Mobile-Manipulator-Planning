package com.github.irvs.ros_tms.tms_ur.tms_ur_refrigerator;

import android.content.Intent;
import android.os.Bundle;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;
import java.util.ArrayList;

public class TmsUrRefrigerator extends RosActivity implements View.OnClickListener,RecognitionListener
{
    private java.lang.Object mSpeechRecognizer;
    private com.github.irvs.ros_tms.tms_ur.tms_ur_refrigerator.RecognitionClient recognitionClient;

    private TextView guiMessage;
    public static TextView guiResult;

    /**
     * Default constructor
     */
    public TmsUrRefrigerator() {
        super("fridge control", "fridge control", URI.create("http://192.168.4.170:11311"));
    }

    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        Button button1 = (Button)findViewById(R.id.button);
        button1.setOnClickListener(this);

        guiMessage = (TextView)findViewById(R.id.message);
        guiResult = (TextView) findViewById(R.id.result);

        /**
         * Generate the instance of SpeechRecognizer, and set the listener.
         */
        mSpeechRecognizer = SpeechRecognizer.createSpeechRecognizer(this/*, ComponentName serviceComponent*/);
        ((SpeechRecognizer) mSpeechRecognizer).setRecognitionListener(this);
    }

    /**
     * Here is a unique part for ROS.
     */
    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        /**
         * Set the Configuration parameter such as the master URI.
         */
        NodeConfiguration nodeConfiguration
            = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());
        /**
         * Generates a new node instance,
         * and executes it on a worker threads.
         */
        recognitionClient = new RecognitionClient();
        nodeMainExecutor.execute(recognitionClient, nodeConfiguration);
    }

    @Override
    public void onClick(View v) {
        /**
         * Executes speech recognition service,
         * when the button selected on UI thread.
         */
        switch (v.getId()) {
            case R.id.button:
                /**
                 * Intent setting for speech recognition service
                 */
                Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
                intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL,
                    RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
                intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, getPackageName());
                /**
                 * Start speech recognition
                 */
                ((SpeechRecognizer) mSpeechRecognizer).startListening(intent);
                break;
            default:
                break;
        }
    }

    //----------------------------------------------------------------------------------------------
    /**
     * Overridden functions for SpeechRecognizer intent
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
        /**
         * Error message
         */
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
                break;
            case SpeechRecognizer.ERROR_RECOGNIZER_BUSY:
                guiMessage.setText("ERROR_RECOGNIZER_BUSY");
                break;
            case SpeechRecognizer.ERROR_SERVER:
                guiMessage.setText("ERROR_SERVER");
                break;
            case SpeechRecognizer.ERROR_SPEECH_TIMEOUT:
                guiMessage.setText("ERROR_SPEECH_TIMEOUT(No input)");
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
        Log.v("DEBUG", "onPartialResults");
    }

    @Override
    public void onResults(Bundle results) {
        /**
         * The ArrayList to receive the candidates of results
         */
        ArrayList<String> recData = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
        final String[] str_items = recData.toArray(new String[recData.size()]);

        Log.i("ROS:TmsUrDemo","Send recognized result to server");
        guiResult.setText(str_items[0]);

        if (str_items[0].equals("open") ||
            str_items[0].equals("open the door") ||
            str_items[0].equals("open the fridge")) {
            /**
             * Send the request to open the fridge in the ROS-TMS
             */
            recognitionClient.sendRequest(1);
        } else if (str_items[0].equals("close") ||
            str_items[0].equals("close the door") ||
            str_items[0].equals("close the fridge")) {
            /**
             * Send the request to close the fridge in the ROS-TMS
             */
            recognitionClient.sendRequest(0);
        }
    }
}
