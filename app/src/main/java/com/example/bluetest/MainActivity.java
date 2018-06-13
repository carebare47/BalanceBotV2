package com.example.bluetest;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;

import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.AsyncTask;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;

import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;

import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.net.SocketImpl;
import java.util.ArrayList;
import java.util.Set;
import java.util.UUID;

public class MainActivity extends Activity {
    Button b1, b2, b3, b4, b5;
    BluetoothAdapter myBluetooth = null;
    public static BluetoothSocket btSocket = null;
    private BluetoothAdapter BA;
    private Set<BluetoothDevice> pairedDevices;
    private boolean isBtConnected = false;
    private String address = null;
    static final UUID myUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    ListView lv;
    public static String EXTRA_ADDRESS = "device_address";
    public static String SOCKET= "device_socket";


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        b1 = (Button) findViewById(R.id.button);
        b2 = (Button) findViewById(R.id.button2);
        b3 = (Button) findViewById(R.id.button3);
        b4 = (Button) findViewById(R.id.button4);
        b5 = (Button) findViewById(R.id.button5);

        BA = BluetoothAdapter.getDefaultAdapter();

        if (BA == null) {
            //Show a mensag. that the device has no bluetooth adapter
            Toast.makeText(getApplicationContext(), "Bluetooth Device Not Available", Toast.LENGTH_LONG).show();

            //finish apk
            finish();
        }

        lv = (ListView) findViewById(R.id.listView);
    }

    //    protected void onPause(Bundle savedInstanceState) {
    public void on(View v) {
        if (!BA.isEnabled()) {
            Intent turnOn = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(turnOn, 0);
            Toast.makeText(getApplicationContext(), "Turned on", Toast.LENGTH_LONG).show();
        } else {
            Toast.makeText(getApplicationContext(), "Already on", Toast.LENGTH_LONG).show();
        }
    }

    public void off(View v) {
        BA.disable();
        Toast.makeText(getApplicationContext(), "Turned off", Toast.LENGTH_LONG).show();
    }

    public void control(View v) {
        // Do something in response to button
        Intent intent = new Intent(this, ControlActivity.class);
        //EditText editText = (EditText) findViewById(R.id.editText);
        //String message = editText.getText().toString();
        //intent.putExtra(EXTRA_MESSAGE, message);
        startActivity(intent);
        //setContentView(R.layout.activity_control);
    }

    public void visible(View v) {
        Intent getVisible = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
        startActivityForResult(getVisible, 0);
    }


    private AdapterView.OnItemClickListener myListClickListener = new AdapterView.OnItemClickListener() {
        public void onItemClick(AdapterView<?> av, View v, int arg2, long arg3) {
            // Get the device MAC address, the last 17 chars in the View
            String info = ((TextView) v).getText().toString();
            //String address = info.substring(info.length() - 17);
            String address = info.substring(info.length() - 17);


            if (address != null) {
                new MainActivity.ConnectBT().execute(); //Call the class to connect
            }

            if (isBtConnected) {
                // Make an intent to start next activity.
                Intent i = new Intent(MainActivity.this, IMUActivity.class);
//                Bundle extras = new Bundle();
//                extras.putString(EXTRA_ADDRESS, address);
//                //extras.putString(SOCKET, btSocket);
//           //     extras.putSerializable(SOCKET, btSocket);
//                //extras.put
//                i.putExtras(extras);
//                startActivity(i);

                //Change the activity.
                i.putExtra(EXTRA_ADDRESS, address ); //this will be received at ledControl (class) Activity
                startActivity(i);
//
/*
                You could pass a 'bundle' of extras rather than individual extras if you like, for example:-

                        Intent intent = new Intent(this, MyActivity.class);
                Bundle extras = new Bundle();
                extras.putString("EXTRA_USERNAME","my_username");
                extras.putString("EXTRA_PASSWORD","my_password");
                intent.putExtras(extras);
                startActivity(intent);

                Then in your Activity that your triggering, you can reference these like so:-

                        Intent intent = getIntent();
                Bundle extras = intent.getExtras();
                String username_string = extras.getString("EXTRA_USERNAME");
                String password_string = extras.getString("EXTRA_PASSWORD");
                */

            }
        }
    };




        public void list(View v) {
            pairedDevices = BA.getBondedDevices();

            ArrayList list = new ArrayList();

            for (BluetoothDevice bt : pairedDevices) {
                list.add(bt.getName() + "\n" + (bt.getAddress()));
                //list.add(bt.getName().concat(bt.getAddress())); //This works
                //list.add(bt.getAddress());
            }
            Toast.makeText(getApplicationContext(), "Showing Paired Devices", Toast.LENGTH_SHORT).show();

            final ArrayAdapter adapter = new ArrayAdapter(this, android.R.layout.simple_list_item_1, list);

            lv.setAdapter(adapter);
            //Call myListClickListener which passes address to next activity
            lv.setOnItemClickListener(myListClickListener); //Method called when the device from the list is clicked
        }





    class ConnectBT extends AsyncTask<Void, Void, Void>  // UI thread
    {
        private boolean ConnectSuccess = true; //if it's here, it's almost connected

        @Override
        protected void onPreExecute() {
            //progress = ProgressDialog.show(ControlActivity.this, "Connecting...", "Please wait!!!");  //show a progress dialog
            msg("Connecting, please wait...");
        }

        @Override
        protected Void doInBackground(Void... devices) //while the progress dialog is shown, the connection is done in background
        {
            try {
                if (btSocket == null || !isBtConnected) {

                    myBluetooth = BluetoothAdapter.getDefaultAdapter();//get the mobile bluetooth device
                    BluetoothDevice dispositivo = myBluetooth.getRemoteDevice(address);//connects to the device's address and checks if it's available

                    btSocket = dispositivo.createInsecureRfcommSocketToServiceRecord(myUUID);//create a RFCOMM (SPP) connection                     // btSocket = dispositivo.createRfcommSocketToServiceRecord(myUUID);
                    BluetoothAdapter.getDefaultAdapter().cancelDiscovery();
                    btSocket.connect();//start connection
                }
            } catch (IOException e) {
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

            if (!ConnectSuccess) {
                msg("Connection Failed. Is it a SPP Bluetooth? Try again.");
                finish();
            } else {
                msg("Connected.");
                isBtConnected = true;
            }
            //  progress.dismiss();
        }
    }

    private void msg(String s) {
        Toast.makeText(getApplicationContext(), s, Toast.LENGTH_LONG).show();
    }
}