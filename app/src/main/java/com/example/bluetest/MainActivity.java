package com.example.bluetest;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;

import android.content.Intent;
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
import java.util.ArrayList;
import java.util.Set;

public class MainActivity extends Activity  {
    Button b1,b2,b3,b4,b5;
    private BluetoothAdapter BA;
    private Set<BluetoothDevice>pairedDevices;
    ListView lv;
    public static String EXTRA_ADDRESS = "device_address";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        b1=(Button)findViewById(R.id.button);
        b2=(Button)findViewById(R.id.button2);
        b3=(Button)findViewById(R.id.button3);
        b4=(Button)findViewById(R.id.button4);
        b5=(Button)findViewById(R.id.button5);

        BA = BluetoothAdapter.getDefaultAdapter();

        if(BA == null)
        {
            //Show a mensag. that the device has no bluetooth adapter
            Toast.makeText(getApplicationContext(), "Bluetooth Device Not Available", Toast.LENGTH_LONG).show();

            //finish apk
            finish();
        }

        lv = (ListView)findViewById(R.id.listView);
    }
//    protected void onPause(Bundle savedInstanceState) {
    public void on(View v){
        if (!BA.isEnabled()) {
            Intent turnOn = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(turnOn, 0);
            Toast.makeText(getApplicationContext(), "Turned on",Toast.LENGTH_LONG).show();
        } else {
            Toast.makeText(getApplicationContext(), "Already on", Toast.LENGTH_LONG).show();
        }
    }

    public void off(View v){
        BA.disable();
        Toast.makeText(getApplicationContext(), "Turned off" ,Toast.LENGTH_LONG).show();
    }

    public void control(View v){
        // Do something in response to button
        Intent intent = new Intent(this, ControlActivity.class);
        //EditText editText = (EditText) findViewById(R.id.editText);
        //String message = editText.getText().toString();
        //intent.putExtra(EXTRA_MESSAGE, message);
        startActivity(intent);
        //setContentView(R.layout.activity_control);
    }
    public  void visible(View v){
        Intent getVisible = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
        startActivityForResult(getVisible, 0);
    }


    private AdapterView.OnItemClickListener myListClickListener = new AdapterView.OnItemClickListener()
    {
        public void onItemClick (AdapterView<?> av, View v, int arg2, long arg3)
        {
            // Get the device MAC address, the last 17 chars in the View
            String info = ((TextView) v).getText().toString();
            //String address = info.substring(info.length() - 17);
            String address = info.substring(info.length() - 17);

            // Make an intent to start next activity.
            Intent i = new Intent(MainActivity.this, ControlActivity.class);

            //Change the activity.
            i.putExtra(EXTRA_ADDRESS, address); //this will be received at ledControl (class) Activity
            startActivity(i);
        }
    };



    public void list(View v){
        pairedDevices = BA.getBondedDevices();

        ArrayList list = new ArrayList();

        for(BluetoothDevice bt : pairedDevices){
            list.add(bt.getName().concat(bt.getAddress()));
            //list.add(bt.getAddress());
        }
        Toast.makeText(getApplicationContext(), "Showing Paired Devices",Toast.LENGTH_SHORT).show();

        final ArrayAdapter adapter = new  ArrayAdapter(this,android.R.layout.simple_list_item_1, list);

        lv.setAdapter(adapter);
        //Call myListClickListener which passes address to next activity
        lv.setOnItemClickListener(myListClickListener); //Method called when the device from the list is clicked
    }

}