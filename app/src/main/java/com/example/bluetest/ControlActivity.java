package com.example.bluetest;

import android.app.Activity;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.content.res.Resources;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.view.View;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.util.UUID;


public class ControlActivity extends Activity {
    //bt timer
    private Handler handler;
    Button b5,b8,b9;
    SeekBar seek1;
    BluetoothAdapter myBluetooth = null;
    private boolean commandReady = false;
    BluetoothSocket btSocket = null;
    BluetoothDevice dispositivo = null;
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
        setContentView(R.layout.activity_control);

        //for bluetooth timer
        handler = new Handler();

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
            new ConnectBT().execute(); //Call the class to connect
        }


        //commands to be sent to bluetooth
        seek1.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                 @Override
                 public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                     if (fromUser==true) {
                         TextView textView = findViewById(R.id.textView5);
                         //textView.setText(String.valueOf((progress/range_precision)-(range_max/2))+"\u00B0");
                         float progressFloat = ((float) progress / (float) range_precision) - ((float) maximumRange / 2);
                         textView.setText("   " + String.valueOf(progressFloat) + "\u00B0");
                         //Uncomment for hc-05
                         if (isBtConnected && commandReady) {
                             try {
                                 //msg("Alter code to connect to hc-05");
                                 btSocket.getOutputStream().write(String.valueOf("&r=" + (progress) + "e").getBytes());
                                 commandReady = false;

                             } catch (IOException e) {
                                 msg("Couldn't send angle message");
                             }
                         }
                     }
                 }

                 @Override
                 public void onStartTrackingTouch(SeekBar seekBar) {

                 }

                 @Override
                 public void onStopTrackingTouch(SeekBar seekBar) {
                     //Snap back to spaghetti
                     seek1.setProgress(seek1.getMax()/2);
                     TextView textView = findViewById(R.id.textView5);
                     textView.setText("0\u00B0");
                     if (isBtConnected) {
                         try {
                             //msg("Alter code to connect to hc-05");
                             btSocket.getOutputStream().write(String.valueOf("&r=500e").getBytes());
                             btSocket.getOutputStream().write(String.valueOf("&k=500e").getBytes());
                             btSocket.getOutputStream().write(String.valueOf("&r=500e").getBytes());
                             btSocket.getOutputStream().write(String.valueOf("&k=500e").getBytes());
                             //commandReady = false;

                         } catch (IOException e) {
                             msg("Couldn't send angle message");
                         }
                     }
                 }
             });




    }

    Runnable mStatusChecker = new Runnable() {
        @Override
        public void run() {
            try {
                commandReady = true;
            } finally {
                // 100% guarantee that this always happens, even if
                // your update method throws an exception
                handler.postDelayed(mStatusChecker, 100);
            }


        }
    };
    void startRepeatingTask() {
        mStatusChecker.run();
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


    public void position(View v){
        // Do something in response to button
        disconnect();
        Intent intent = new Intent(this, PositionActivity.class);
        startActivity(intent);
    }


    public void angle(View v){
        disconnect();
        // Do something in response to button
        Intent intent = new Intent(this, ControlActivity.class);
        startActivity(intent);
    }


    public void imu(View v){
        disconnect();
        // Do something in response to button
        Intent intent = new Intent(this, IMUActivity.class);
        startActivity(intent);
    }
    // fast way to call Toast
    private void msg(String s)
    {
        Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }

    private void turnOnLed()
    {
        if (btSocket!=null)
        {
            try
            {
                btSocket.getOutputStream().write("TO".toString().getBytes());
            }
            catch (IOException e)
            {
                msg("Error");
            }
        }
    }




    private class ConnectBT extends AsyncTask<Void, Void, Void>  // UI thread
    {
        private boolean ConnectSuccess = true; //if it's here, it's almost connected
        private boolean NoDevice = false; //if it's here, it's almost connected
        private boolean NonSPPDevice = false; //if it's here, it's almost connected

        @Override
        protected void onPreExecute()
        {
           //progress = ProgressDialog.show(ControlActivity.this, "Connecting...", "Please wait!!!");  //show a progress dialog
        }

        @Override
        protected Void doInBackground(Void... devices) //while the progress dialog is shown, the connection is done in background
        {
            try
            {
                if (btSocket == null || !isBtConnected) {

                        myBluetooth = BluetoothAdapter.getDefaultAdapter();//get the mobile bluetooth device
                        dispositivo = myBluetooth.getRemoteDevice(address);//connects to the device's address and checks if it's available

                        btSocket = dispositivo.createInsecureRfcommSocketToServiceRecord(myUUID);//create a RFCOMM (SPP) connection //use this for the hc-05
                       // btSocket = dispositivo.createRfcommSocketToServiceRecord(myUUID);
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
                startRepeatingTask();
            }
          //  progress.dismiss();
        }
    }
}
//    msg("Connection Failed. Device unreachable.");
//    finish();
//    msg("Connection Failed. Are you sure you're connecting to the robot?.");
//    finish();