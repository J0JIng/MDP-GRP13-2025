package com.example.mdp_android.controllers;

import android.content.Context;
import android.os.Handler;

public class BluetoothControllerSingleton {
    private static BluetoothController instance;

    // Existing 1-arg accessor (kept for compatibility)
    public static synchronized BluetoothController getInstance(Handler handler) {
        if (instance == null) {
            instance = new BluetoothController(handler);
        } else if (handler != null) {
            instance.setHandler(handler);
        }
        return instance;
    }

    // Preferred 2-arg accessor (enables broadcasts)
    public static synchronized BluetoothController getInstance(Context appContext, Handler handler) {
        if (instance == null) {
            instance = new BluetoothController(appContext, handler);
        } else {
            instance.setAppContext(appContext);
            if (handler != null) instance.setHandler(handler);
        }
        return instance;
    }
}
