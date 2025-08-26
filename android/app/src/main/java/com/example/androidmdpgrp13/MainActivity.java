package com.example.androidmdpgrp13;

import android.os.Bundle;
import androidx.appcompat.app.AppCompatActivity;
import com.google.android.material.appbar.MaterialToolbar;
import com.google.android.material.bottomnavigation.BottomNavigationView;

import androidx.navigation.NavController;
import androidx.navigation.fragment.NavHostFragment;
import androidx.navigation.ui.NavigationUI;

public class MainActivity extends AppCompatActivity {

    @Override protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // 1) Toolbar
        MaterialToolbar toolbar = findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        // 2) Get NavController SAFELY via NavHostFragment
        NavHostFragment navHostFragment =
                (NavHostFragment) getSupportFragmentManager().findFragmentById(R.id.nav_host);
        if (navHostFragment == null) {
            throw new IllegalStateException("NavHostFragment (R.id.nav_host) not found. Check activity_main.xml.");
        }
        NavController navController = navHostFragment.getNavController();

        // 3) Wire BottomNavigationView
        BottomNavigationView bottom = findViewById(R.id.bottom_nav);
        NavigationUI.setupWithNavController(bottom, navController);
    }
}
