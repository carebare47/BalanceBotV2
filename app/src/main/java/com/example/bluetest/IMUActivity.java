package com.example.bluetest;

import android.app.Activity;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.AsyncTask;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.util.UUID;

public class IMUActivity extends Activity {
    Button b5,b8,b9;
    SeekBar seek1;
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

        b5=(Button)findViewById(R.id.button5);
        b8=(Button)findViewById(R.id.button8);
        b9=(Button)findViewById(R.id.button9);
        seek1=(SeekBar)findViewById(R.id.seekBar2);
        final int maximumRange = this.getResources().getInteger(R.integer.range_max);

        seek1.setMax(maximumRange*range_precision);

        //Initialise seekbar to 50%

        seek1.setProgress(seek1.getMax()/2);

        Intent newint = getIntent();
        address = newint.getStringExtra(MainActivity.EXTRA_ADDRESS); //receive the address of the bluetooth device
        if (address != null) {
            new IMUActivity.ConnectBT().execute(); //Call the class to connect
        }
        //commands to be sent to bluetooth
        seek1.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                if (fromUser==true)
                {
                    TextView textView = findViewById(R.id.textView5);
                    //textView.setText(String.valueOf((progress/range_precision)-(range_max/2))+"\u00B0");
                    float progressFloat = ((float)progress/(float)range_precision) - ((float) maximumRange / 2);
                    textView.setText("   " + String.valueOf(progressFloat)+"\u00B0");
                    //Uncomment for hc-05
//                         try
//                         {
                    msg("Alter code to connect to hc-05");
                    //btSocket.getOutputStream().write(String.valueOf(progress).getBytes());
//                         }
//                         catch (IOException e)
//                         {
//                             msg("Couldn't send angle message");
//                         }
                }
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });


    }
//    public class SensorActivity extends Activity implements SensorEventListener {
//        private final SensorManager mSensorManager;
//        private final Sensor mAccelerometer;
//        setContentView(R.layout.activity_position);
//
//        public SensorActivity() {
//            mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
//            mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
//        }
//
//        protected void onResume() {
//            super.onResume();
//            mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
//        }
//
//        protected void onPause() {
//            super.onPause();
//            mSensorManager.unregisterListener(this);
//        }
//
//        public void onAccuracyChanged(Sensor sensor, int accuracy) {
//        }
//
//        public void onSensorChanged(SensorEvent event) {
//            float[] mGravity;
//            float[] mGeomagnetic;
//            if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
//                mGravity = event.values;
//            if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
//                mGeomagnetic = event.values;
//            if ((mGravity == null) || (mGeomagnetic == null))
//                return;
//
//            float[] R = new float[9];
//            float[] I = new float[9];
//            if (!SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic))
//                return;
//
//            float[] orientation = new float[3];
//            SensorManager.getOrientation(R, orientation);
//
//            if (orientation == null)
//                return;
//
//            double rollAngle = orientation[1] * 180 / Math.PI;
//            double pitchAngle = orientation[2] * 180 / Math.PI;
//
//            TextView textView = findViewById(R.id.textView5);
//            textView.setText("   " + String.valueOf(rollAngle)+"\u00B0");
//
//        }
//    }


    private class ConnectBT extends AsyncTask<Void, Void, Void>  // UI thread
    {
        private boolean ConnectSuccess = true; //if it's here, it's almost connected
        private boolean NoDevice = false; //if it's here, it's almost connected
        private boolean NonSPPDevice = false; //if it's here, it's almost connected

        @Override
        protected void onPreExecute()
        {
            progress = ProgressDialog.show(IMUActivity.this, "Connecting...", "Please wait!!!");  //show a progress dialog
        }

        @Override
        protected Void doInBackground(Void... devices) //while the progress dialog is shown, the connection is done in background
        {
            try
            {
                if (btSocket == null || !isBtConnected) {

                    myBluetooth = BluetoothAdapter.getDefaultAdapter();//get the mobile bluetooth device
                    BluetoothDevice dispositivo = myBluetooth.getRemoteDevice(address);//connects to the device's address and checks if it's available

                    //btSocket = dispositivo.createInsecureRfcommSocketToServiceRecord(myUUID);//create a RFCOMM (SPP) connection //use this for the hc-05
                    btSocket = dispositivo.createRfcommSocketToServiceRecord(myUUID);
                    BluetoothAdapter.getDefaultAdapter().cancelDiscovery();
                    btSocket.connect();//start connection
                }
            }
            catch (IOException e)
            {
                ConnectSuccess = false;//if the try failed, you can check the exception here
            }
            return null;
        }
        @Override
        protected void onPostExecute(Void result) //after the doInBackground, it checks if everything went fine
        {
            super.onPostExecute(result);

            //NonSPPDevice

            //NoDevice

            if (!ConnectSuccess)
            {
                msg("Connection Failed. Is it a SPP Bluetooth? Try again.");
                finish();
            }
            else
            {
                msg("Connected.");
                isBtConnected = true;
            }
            //  progress.dismiss();
        }
    }


    // fast way to call Toast
    private void msg(String s)
    {
        Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }

    public void position(View v){
        // Do something in response to button
        Intent intent = new Intent(this, IMUActivity.class);
        startActivity(intent);
    }


    public void angle(View v){
        // Do something in response to button
        Intent intent = new Intent(this, ControlActivity.class);
        startActivity(intent);
    }

}
