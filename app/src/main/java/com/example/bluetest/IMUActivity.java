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
import android.view.View;
import android.widget.Button;
import android.widget.ProgressBar;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import java.io.IOException;
import java.util.UUID;

public class IMUActivity extends Activity implements SensorEventListener{

    private SensorManager sensorManager;
    private Sensor rotation_vector;

    private float fRoll = 0;
    private float fPitch = 0;

    private TextView tRoll, tPitch;

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
        initializeViews();

        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) != null) {
            // success! we have an rotation_vector

            rotation_vector = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
            sensorManager.registerListener(this, rotation_vector, SensorManager.SENSOR_DELAY_NORMAL);
        } else {
            // fai! we dont have an rotation_vector!
        }

        b5=(Button)findViewById(R.id.button5);
        b8=(Button)findViewById(R.id.button8);
        b9=(Button)findViewById(R.id.button9);




        //Initialise seekbar to 50%



//        Intent newint = getIntent();
  //      address = newint.getStringExtra(MainActivity.EXTRA_ADDRESS); //receive the address of the bluetooth device


    }

    public void initializeViews() {
        tRoll = (TextView) findViewById(R.id.tRoll);
        tPitch = (TextView) findViewById(R.id.tPitch);
    }




    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    @Override
    public void onSensorChanged(SensorEvent event) {
    // clean current values
    displayCleanValues();
    // display the current x,y rotation_vector values
    displayCurrentValues();

    fPitch = event.values[0];
    fRoll= event.values[1];

        //Uncomment for hc-05
        try
        {
            msg("Alter code to connect to hc-05");
            btSocket.getOutputStream().write(String.valueOf("pitch=" + fPitch + "roll=" + fRoll).getBytes());
        }
        catch (IOException e)
        {
            msg("Couldn't send angle message");
        }


    }

    public void displayCleanValues() {
        tRoll.setText("0.0");
        tPitch.setText("0.0");
    }

    // display the current x,y,z rotation_vector values
    public void displayCurrentValues() {
        tRoll.setText(Float.toString(fRoll));
        tPitch.setText(Float.toString(fPitch));
    }


    


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
