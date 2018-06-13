package com.example.bluetest;

import android.app.Activity;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.view.View;
import android.widget.Button;
import android.widget.ProgressBar;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import com.google.gson.Gson;

import java.io.IOException;
import java.util.UUID;

public class IMUActivity extends Activity implements SensorEventListener {

    private SensorManager sensorManager;
    private Sensor rotation_vector;

    public static String EXTRA_ADDRESS = "device_address";
    public static String SOCKET= "device_socket";
    private float fRoll = 0;
    private float fPitch = 0;
    private boolean txFlag = false;

    private TextView tRoll, tPitch, sensitivityText;
    private SeekBar sensitivity;

    Button b5, b8, b9;
    BluetoothAdapter myBluetooth = null;
    BluetoothSocket btSocket = null;
    String address = null;
    //Integer range_max = this.getResources().getInteger(R.integer.range_max);
    //Integer range_min = R.integer.range_min;
    Integer range_precision = 100;
    private boolean isBtConnected = false;
    private ProgressDialog progress;
    static final UUID myUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_imu);
        initializeViews();

        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) != null) {
            // success! we have an rotation_vector

            rotation_vector = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
            sensorManager.registerListener(this, rotation_vector, SensorManager.SENSOR_DELAY_NORMAL);
        } else {
            // fai! we dont have an rotation_vector!
        }

        b5 = (Button) findViewById(R.id.button5);
        b8 = (Button) findViewById(R.id.button8);
        b9 = (Button) findViewById(R.id.button9);
        sensitivity = (SeekBar) findViewById(R.id.sensitivity);


        //Initialise seekbar to 50%
        sensitivity.setMax(100);
        sensitivity.setProgress((int) (sensitivity.getProgress() / 2));

        Intent newint = getIntent();
        //address = newint.getStringExtra(MainActivity.EXTRA_ADDRESS); //receive the address of the bluetooth device

        String jsonMyObject = null;
        Bundle extras = getIntent().getExtras();
        if (extras != null) {
            jsonMyObject = extras.getString(SOCKET);
        }
        btSocket = new Gson().fromJson(jsonMyObject, BluetoothSocket.class);


//        if (address != null) {
//            new IMUActivity.ConnectBT().execute(); //Call the class to connect
//        }

       // new IMUActivity.TransmitAngle().execute();


        sensitivity.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                if (fromUser == true) {
                    TextView textView = findViewById(R.id.sensitivityText);
                    //textView.setText(String.valueOf((progress/range_precision)-(range_max/2))+"\u00B0");
                    //float progressFloat = ((float)progress/(float)range_precision) - ((float) maximumRange / 2);
                    //float progressFloat = ((float)progress - ((float)positionRange/2))/(float)range_precision;///(float)range_precision) - ((float) maximumRange / 2);
                    //
                    String str = ("   " + (progress - 50));
                    textView.setText(str);
                    //textView.setText(String.valueOf("   " + (progress - 50)));
                   // try {

                        //btSocket.getOutputStream().write(str.getBytes());
                    try {
                        btSocket.getOutputStream().write(String.valueOf("&p=999&r=999&s=" + progress).getBytes());
                    }
                    catch (IOException e){
                        msg("Can't send sensitivity via bluetooth :c");
                    }

                }
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
//        Intent newint = getIntent();
        //      address = newint.getStringExtra(MainActivity.EXTRA_ADDRESS); //receive the address of the bluetooth device





    }

    public void initializeViews() {
        tRoll = (TextView) findViewById(R.id.tRoll);
        tPitch = (TextView) findViewById(R.id.tPitch);
        sensitivityText = (TextView) findViewById(R.id.sensitivityText);
    }


    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    @Override
    public void onSensorChanged(SensorEvent event) {

        final Handler UI_update_handler = new Handler();
        Runnable UI_update = new Runnable() {
            @Override
            public void run() {
                //Uncomment for hc-05
                // clean current values
                displayCleanValues();
                // display the current x,y rotation_vector values
                displayCurrentValues();

                UI_update_handler.postDelayed(this, 1000);
            }
        };
        UI_update_handler.post(UI_update);


        fPitch = event.values[0];
        fRoll = event.values[1];

        txFlag = true;

      //  if (isBtConnected) {
                    try {
                        //  msg("Alter code to connect to hc-05");
                        btSocket.getOutputStream().write(String.valueOf("&p=" + (fPitch) + "&r=" + (fRoll)+ "&s=999").getBytes());
                        //btSocket.getOutputStream().write(String.valueOf("pitch=" + fPitch + "roll=" + fRoll).getBytes());
                    } catch (IOException e) {
                        msg("Couldn't send angle message");
                    }
        //        }


//
//        final Handler handler = new Handler();
//        Runnable task = new Runnable() {
//            @Override
//            public void run() {
//                //Uncomment for hc-05
//                if (isBtConnected) {
//                    try {
//                        //  msg("Alter code to connect to hc-05");
//                        btSocket.getOutputStream().write(String.valueOf("pitch=" + fPitch + "roll=" + fRoll + "\n").getBytes());
//                        //btSocket.getOutputStream().write(String.valueOf("pitch=" + fPitch + "roll=" + fRoll).getBytes());
//                    } catch (IOException e) {
//                        msg("Couldn't send angle message");
//                    }
//                }
//                handler.postDelayed(this, 200);
//            }
//        };
//        handler.post(task);


    }

//        private class TransmitAngle extends AsyncTask<Void, Void, Void> {
//
//            @Override
//            protected Void doInBackground(Void... devices) {
//                if (isBtConnected && txFlag) {
//                    try {
//                        //  msg("Alter code to connect to hc-05");
//                        btSocket.getOutputStream().write(String.valueOf("pitch=" + fPitch + "roll=" + fRoll + "\n").getBytes());
//                        //btSocket.getOutputStream().write(String.valueOf("pitch=" + fPitch + "roll=" + fRoll).getBytes());
//                    } catch (IOException e) {
//                        msg("Couldn't send angle message");
//                    }
//                }
//                txFlag = false;
//                return null;
//            }
//
//            protected void onProgressUpdate() {
//                //setProgressPercent(progress[0]);
//                msg("Message sent:");
//
//            }
//
//            protected void onPostExecute(Void result) {
//                super.onPostExecute(result);
//                //showDialog("Downloaded " + result + " bytes");
//                msg("post");
//            }
//        }






    public void displayCleanValues() {
        tRoll.setText("0.0");
        tPitch.setText("0.0");
    }

    // display the current x,y,z rotation_vector values
    public void displayCurrentValues() {
        tRoll.setText(Float.toString(fRoll));
        tPitch.setText(Float.toString(fPitch));
    }




    private void disconnect() {
//        if (mBTInputStream != null) {
//            try {mBTInputStream.close();} catch (Exception e) {}
//            mBTInputStream = null;
//        }
//
//        if (mBTOutputStream != null) {
//            try {mBTOutputStream.close();} catch (Exception e) {}
//            mBTOutputStream = null;
//        }

        if (btSocket != null) {
            try {btSocket.close();} catch (Exception e) {}
            btSocket = null;
        }

    }


    // fast way to call Toast
    private void msg(String s) {
        Toast.makeText(getApplicationContext(), s, Toast.LENGTH_LONG).show();
    }

    public void position(View v) {
        // Do something in response to button
        Intent i = new Intent(IMUActivity.this, PositionActivity.class);

        //Change the activity.
        i.putExtra(EXTRA_ADDRESS, address); //this will be received at ledControl (class) Activity
        startActivity(i);

        //Intent intent = new Intent(this, IMUActivity.class);
        //startActivity(intent);
    }


    public void angle(View v) {
        //disconnect();
        Intent i = new Intent(IMUActivity.this, ControlActivity.class);

        //Change the activity.
        i.putExtra(EXTRA_ADDRESS, address); //this will be received at ledControl (class) Activity
        startActivity(i);

        // Do something in response to button
//        Intent intent = new Intent(this, ControlActivity.class);
  //      startActivity(intent);
    }

}
