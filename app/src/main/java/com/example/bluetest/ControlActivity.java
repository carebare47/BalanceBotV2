package com.example.bluetest;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.os.Bundle;
import android.widget.Button;


public class ControlActivity extends Activity {
    Button b5,b8,b9;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_control);

        b5=(Button)findViewById(R.id.button5);
        b8=(Button)findViewById(R.id.button8);
        b9=(Button)findViewById(R.id.button9);

    }
}
