package com.example.mdp_android;

import android.app.Application;
import android.content.Context;

public class MyApp extends Application {
    private static MyApp s;
    @Override public void onCreate() { super.onCreate(); s = this; }
    public static Context get() { return s.getApplicationContext(); }
}
