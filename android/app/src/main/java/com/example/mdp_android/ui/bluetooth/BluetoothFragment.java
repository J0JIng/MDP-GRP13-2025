package com.example.mdp_android.ui.bluetooth;

import static com.example.mdp_android.controllers.BluetoothController.DEVICE_NAME;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.provider.Settings;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.ProgressBar;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.Observer;
import androidx.lifecycle.ViewModelProvider;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import com.airbnb.lottie.LottieAnimationView;
import com.airbnb.lottie.LottieDrawable;
import com.example.mdp_android.R;
import com.example.mdp_android.controllers.BluetoothController;
import com.example.mdp_android.controllers.BluetoothControllerSingleton;
import com.example.mdp_android.controllers.DeviceSingleton;
import com.example.mdp_android.databinding.FragmentBluetoothBinding;

import java.util.Set;

public class BluetoothFragment extends Fragment {

    private static final String TAG = "BluetoothFragment";

    private BluetoothViewModel bluetoothViewModel;
    private FragmentBluetoothBinding binding;

    private ListView lvPairedDevices;
    private ListView lvAvailableDevices;
    private ArrayAdapter<String> aPairedDevices;
    private ArrayAdapter<String> aAvailableDevices;
    private Button scanBtn;
    private ProgressBar progressAvail;
    private TextView bluetoothTextView;
    private BluetoothAdapter bAdapter;
    //public BluetoothController bController = BluetoothControllerSingleton.getInstance(null);
    private BluetoothController bController; // just a field
    private LottieAnimationView lottieScan;
    private boolean isScanning = false;
    private boolean receiversRegistered = false;

    // bluetooth indicators
    private String connectedDevice = "";
    DeviceSingleton deviceSingleton;

    @Override
    public void onAttach(@NonNull Context ctx) {
        super.onAttach(ctx);
        Context app = ctx.getApplicationContext();
        bController = BluetoothControllerSingleton.getInstance(app, mHandler);
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        bluetoothViewModel = new ViewModelProvider(this).get(BluetoothViewModel.class);
        binding = FragmentBluetoothBinding.inflate(inflater, container, false);
        View root = binding.getRoot();

        root.setBackgroundResource(R.drawable.background_pattern);

        // init adapter FIRST (so we can use it below)
        bAdapter = BluetoothAdapter.getDefaultAdapter();

        // find views
        lvPairedDevices = root.findViewById(R.id.lvPaired);
        lvAvailableDevices = root.findViewById(R.id.lvAvailable);
        scanBtn = root.findViewById(R.id.button_scan);
        progressAvail = root.findViewById(R.id.progressAvailable);
        bluetoothTextView = root.findViewById(R.id.textView_bluetooth);
        lottieScan = root.findViewById(R.id.lottieScan);

        if (lottieScan != null) {
            lottieScan.setRepeatCount(LottieDrawable.INFINITE);
            lottieScan.setVisibility(View.VISIBLE); // always visible
            lottieScan.pauseAnimation();            // not moving
            lottieScan.setProgress(0f);             // show first frame (static)
        }

        // bluetooth toggle switch
        Switch bluetoothSwitch = root.findViewById(R.id.switchBluetooth);
        if (bluetoothSwitch != null) {
            bluetoothSwitch.setChecked(bAdapter != null && bAdapter.isEnabled());
            bluetoothSwitch.setOnCheckedChangeListener((btn, isChecked) -> {
                if (bAdapter == null) {
                    toast("Bluetooth not supported", Toast.LENGTH_SHORT);
                    btn.setChecked(false);
                    return;
                }
                if (isChecked) {
                    if (!bAdapter.isEnabled()) {
                        Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                        startActivity(enableBtIntent);
                        toast("Requesting to turn on Bluetooth...", Toast.LENGTH_SHORT);
                    }
                } else {
                    if (bAdapter.isEnabled()) {
                        // NOTE: On many modern Android versions this may be blocked unless privileged
                        boolean ok = bAdapter.disable();
                        if (!ok) toast("Could not turn off via app. Try system settings.", Toast.LENGTH_SHORT);
                    }
                }
            });
        }

        // Stop button
        Button stopBtn = root.findViewById(R.id.button_stop_scan);
        if (stopBtn != null) {
            stopBtn.setOnClickListener(v -> {
                if (bAdapter != null && bAdapter.isDiscovering()) bAdapter.cancelDiscovery();
                isScanning = false;
                progressAvail.setVisibility(View.GONE);
                if (lottieScan != null) {
                    lottieScan.pauseAnimation();
                    lottieScan.setProgress(0f);
                    lottieScan.setVisibility(View.VISIBLE);
                }
                toast("Stop scanning for surrounding devices", Toast.LENGTH_SHORT);
            });
        }

        // lists/adapters
        aPairedDevices = new ArrayAdapter<>(requireContext(), R.layout.device_item);
        lvPairedDevices.setAdapter(aPairedDevices);
        aAvailableDevices = new ArrayAdapter<>(requireContext(), R.layout.device_item);
        lvAvailableDevices.setAdapter(aAvailableDevices);

        lvPairedDevices.setOnItemClickListener(handleDeviceClicked);
        lvAvailableDevices.setOnItemClickListener(handleDeviceClicked);

        deviceSingleton = DeviceSingleton.getInstance();

        if (bAdapter == null) {
            toast("bluetooth not supported", Toast.LENGTH_SHORT);
        } else {
            if (!bAdapter.isEnabled()) {
                startActivity(new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE));
            } else {
                updatePairedDevices();
            }
        }

        scanBtn.setOnClickListener(view -> updateAvailableDevices());

        return root;
    }

    // hydrating the view with the view model
    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        // make sure controller uses THIS handler only while view exists
        bController.setHandler(mHandler);
    }

    @Override
    public void onStart() {
        super.onStart();
        registerReceivers(); // register only when attached
    }

    @Override
    public void onStop() {
        super.onStop();
        unregisterReceivers(); // pair with onStart
    }

    @Override
    public void onResume() {
        super.onResume();

        // only force static if not currently scanning
        if (!isScanning && lottieScan != null) {
            lottieScan.setVisibility(View.VISIBLE);
            lottieScan.pauseAnimation();
            lottieScan.setProgress(0f);
        }

        bluetoothTextView.setText(bluetoothViewModel.getDevice().getValue());
        bluetoothViewModel.getDevice().observe(getViewLifecycleOwner(), new Observer<String>() {
            @Override
            public void onChanged(@Nullable String s) {
                bluetoothTextView.setText(s);
            }
        });
    }

    @Override
    public void onPause() {
        super.onPause();
        if (!isScanning && lottieScan != null) {
            lottieScan.pauseAnimation();
            lottieScan.setProgress(0f); // ensure static when leaving
        }
    }

    private final AdapterView.OnItemClickListener handleDeviceClicked = (adapterView, view, i, l) -> {
        // get MAC address (last 17 char)
        String info = ((TextView) view).getText().toString();
        String address = info.substring(info.length() - 17);
        String currentDevice = info.substring(0, info.length() - address.length());
        deviceSingleton.setDeviceName(currentDevice);
        connectDevice(address, currentDevice);
    };

    public void connectDevice(String address, String device) {
        final BluetoothDevice bDevice = bAdapter.getRemoteDevice(address);
        bController.setHandler(mHandler);
        if (bAdapter != null && bAdapter.isDiscovering()) bAdapter.cancelDiscovery(); // avoid race
        bController.connect(bDevice);
        connectedDevice = device;
    }

    // Lifecycle-safe handler: drops messages if fragment is not attached or view is gone
    public final Handler mHandler = new Handler(Looper.getMainLooper(), new Handler.Callback() {
        @Override
        public boolean handleMessage(@NonNull Message message) {
            if (!isAdded() || binding == null) {
                Log.w(TAG, "Dropping BT msg " + message.what + " (fragment not attached)");
                return true;
            }

            switch (message.what) {
                case BluetoothController.MessageConstants.MESSAGE_STATE_CHANGE:
                    switch (message.arg1) {
                        case BluetoothController.StateConstants.STATE_NONE:
                            Log.d(TAG + " Handler Log: ", "STATE_NONE");
                            connectedDevice = "";
                            deviceSingleton.setDeviceName(connectedDevice);
                            if (getContext() != null) {
                                bluetoothViewModel.setDevice(getString(R.string.bluetooth_device_connected_not));
                                sendBluetoothStatus(connectedDevice);
                            }
                            break;
                        case BluetoothController.StateConstants.STATE_LISTEN:
                            Log.d(TAG + " Handler Log: ", "STATE_LISTEN");
                            connectedDevice = "";
                            deviceSingleton.setDeviceName(connectedDevice);
                            if (getContext() != null) {
                                bluetoothViewModel.setDevice(getString(R.string.bluetooth_device_connected_not));
                                sendBluetoothStatus(connectedDevice);
                            }
                            break;
                        case BluetoothController.StateConstants.STATE_CONNECTING:
                            Log.d(TAG + " Handler Log: ", "STATE_CONNECTING");
                            toast("Connecting...Please wait", Toast.LENGTH_SHORT);
                            connectedDevice = "";
                            deviceSingleton.setDeviceName(connectedDevice);
                            if (getContext() != null) {
                                bluetoothViewModel.setDevice(getString(R.string.bluetooth_device_connected_not));
                                sendBluetoothStatus(connectedDevice);
                            }
                            break;
                        case BluetoothController.StateConstants.STATE_CONNECTED:
                            Log.d(TAG + " Handler Log: ", "STATE_CONNECTED");
                            toast("connected to: " + connectedDevice, Toast.LENGTH_SHORT);
                            if (getContext() != null) {
                                Log.d(TAG, "update bluetooth status");
                                bluetoothViewModel.setDevice(getString(R.string.bluetooth_device_connected) + connectedDevice);
                                sendBluetoothStatus(connectedDevice);
                            }
                            break;
                        case BluetoothController.StateConstants.STATE_DISCONNECTED:
                            Log.d("Handler Log: ", "STATE_DISCONNECTED");
                            Log.d(TAG, "Connection lost, attempting for reconnection...");
                            break;
                    }
                    break;
                case BluetoothController.MessageConstants.MESSAGE_WRITE:
                    Log.d(TAG + " Handler Log: ", "MESSAGE_WRITE");
                    break;
                case BluetoothController.MessageConstants.MESSAGE_READ:
                    Log.d(TAG + " Handler Log: ", "MESSAGE_READ");
                    byte[] readBuf = (byte[]) message.obj;          // controller sends byte[]
                    int len = message.arg1;                          // number of bytes read
                    if (readBuf == null || len <= 0) {
                        Log.w(TAG, "Empty payload, ignoring");
                        return true;
                    }
                    String readMessage = new String(readBuf, 0, len);
                    sendReceived(readMessage);                       // lifecycle-safe now
                    Log.d(TAG, "Handler Log: MESSAGE_READ - " + readMessage);
                    break;
                case BluetoothController.MessageConstants.MESSAGE_DEVICE_NAME:
                    Log.d(TAG + " Handler Log: ", "MESSAGE_DEVICE_NAME");
                    connectedDevice = message.getData().getString("device_name");
                    if (connectedDevice != null) {
                        deviceSingleton.setDeviceName(connectedDevice);
                    }
                    break;
                case BluetoothController.MessageConstants.MESSAGE_TOAST:
                    Log.d(TAG + " Handler Log: ", "MESSAGE_TOAST");
                    if (getContext() != null) {
                        String error = message.getData().getString("toast");
                        if (error != null) toast(error, Toast.LENGTH_SHORT);
                    }
                    break;
                case BluetoothController.MessageConstants.MESSAGE_PICTURE:
                    Log.d(TAG + " Handler Log: ", "MESSAGE_PICTURE");
                    break;
            }
            return true;
        }
    });

    public void updatePairedDevices() {
        aPairedDevices.clear();
        checkBluetoothPermission();
        Set<BluetoothDevice> pairedDevices = bAdapter.getBondedDevices();
        if (pairedDevices.size() > 0) {
            for (BluetoothDevice d : pairedDevices) {
                aPairedDevices.add(d.getName() + "\n" + d.getAddress());
            }
        }
    }

    public void updateAvailableDevices() {
        if (bAdapter == null) {
            toast("Bluetooth not supported", Toast.LENGTH_SHORT);
            return;
        }
        if (!bAdapter.isEnabled()) {
            toast("Please turn on Bluetooth to scan", Toast.LENGTH_SHORT);
            return;
        }

        aAvailableDevices.clear();
        isScanning = true;

        progressAvail.setVisibility(View.VISIBLE);
        if (lottieScan != null) {
            lottieScan.setVisibility(View.VISIBLE);
            lottieScan.playAnimation();
        }

        if (binding != null) {
            bluetoothViewModel.setDevice("Bluetooth: Scanning…");
        }

        checkBluetoothPermission();

        if (bAdapter.isDiscovering()) {
            bAdapter.cancelDiscovery();
        }
        if (binding != null && getContext() != null) {
            bluetoothViewModel.setDevice("Bluetooth: Scanning…");
            toast("Scanning for surrounding devices...", Toast.LENGTH_SHORT);
        }
        bAdapter.startDiscovery();

    }

    public boolean checkNewDeviceExist(String newDevice) {
        for (int i = 0; i < aAvailableDevices.getCount(); i++) {
            if (newDevice.equals(aAvailableDevices.getItem(i))) {
                return true;
            }
        }
        return false;
    }

    // bluetooth broadcast receiver
    private final BroadcastReceiver bluetoothBroadcastReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            BluetoothDevice bDevice;
            switch (action) {
                case BluetoothDevice.ACTION_FOUND:
                    bDevice = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);

                    // safer: allow nameless devices
                    String name = (bDevice != null ? bDevice.getName() : null);
                    if (name == null) name = "Unknown Device";
                    String address = (bDevice != null ? bDevice.getAddress() : "00:00:00:00:00:00");
                    String newDevice = name + "\n" + address;

                    if (!checkNewDeviceExist(newDevice)
                            && bDevice != null
                            && bDevice.getBondState() != BluetoothDevice.BOND_BONDED) {
                        aAvailableDevices.add(newDevice);
                    }
                    break;

                case BluetoothAdapter.ACTION_DISCOVERY_FINISHED:
                    isScanning = false;
                    progressAvail.setVisibility(View.GONE);
                    if (lottieScan != null) {
                        lottieScan.pauseAnimation();   // stop moving
                        lottieScan.setProgress(0f);    // show static first frame
                        lottieScan.setVisibility(View.VISIBLE); // keep visible
                    }
                    // reset header status here
                    if (binding != null) {
                        bluetoothViewModel.setDevice(getString(R.string.bluetooth_device_connected_not));
                    }
                    Log.d(TAG, "scan complete");
                    break;

                case BluetoothDevice.ACTION_BOND_STATE_CHANGED:
                    Log.d(TAG, "bReceiver: ACTION_BOND_STATE_CHANGED");
                    bDevice = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                    if (bDevice != null && bDevice.getBondState() == BluetoothDevice.BOND_BONDED) {
                        updatePairedDevices();
                    }
                    break;

                case BluetoothAdapter.ACTION_STATE_CHANGED:
                    int state = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, BluetoothAdapter.ERROR);
                    switch (state) {
                        case BluetoothAdapter.STATE_TURNING_ON:
                            toast("Turning Bluetooth on...", Toast.LENGTH_SHORT);
                            break;
                        case BluetoothAdapter.STATE_ON:
                            toast("Bluetooth is ON", Toast.LENGTH_SHORT);
                            break;
                        case BluetoothAdapter.STATE_TURNING_OFF:
                            toast("Turning Bluetooth off...", Toast.LENGTH_SHORT);
                            break;
                        case BluetoothAdapter.STATE_OFF:
                            toast("Bluetooth is OFF", Toast.LENGTH_SHORT);
                            break;
                    }
                    break;

                case BluetoothDevice.ACTION_ACL_DISCONNECTED:
                    Log.d(TAG, "bReceiver: ACTION_ACL_DISCONNECTED");
                    break;
            }
        }
    };

    /** Register receivers only when the Fragment is attached */
    public void registerReceivers() {
        if (receiversRegistered) return;
        Context ctx = getContext();
        if (ctx == null) return;
        IntentFilter filter = new IntentFilter();
        filter.addAction(BluetoothDevice.ACTION_FOUND);
        filter.addAction(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
        filter.addAction(BluetoothDevice.ACTION_ACL_DISCONNECTED);
        filter.addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
        ctx.registerReceiver(bluetoothBroadcastReceiver, filter);
        receiversRegistered = true;
    }

    public void unregisterReceivers() {
        if (!receiversRegistered) return;
        Context ctx = getContext();
        if (ctx == null) return;
        try {
            ctx.unregisterReceiver(bluetoothBroadcastReceiver);
        } catch (IllegalArgumentException ignored) {}
        receiversRegistered = false;
    }

    // Check for BT Permission (safer per-permission check)
    private void checkBluetoothPermission() {
        Context ctx = getContext();
        if (ctx == null) return;

        if (ContextCompat.checkSelfPermission(ctx, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED
                || ContextCompat.checkSelfPermission(ctx, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED
                || ContextCompat.checkSelfPermission(ctx, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED
                || ContextCompat.checkSelfPermission(ctx, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {

            if (isAdded()) {
                ActivityCompat.requestPermissions(requireActivity(),
                        new String[]{
                                Manifest.permission.BLUETOOTH_SCAN,
                                Manifest.permission.BLUETOOTH_CONNECT,
                                Manifest.permission.ACCESS_FINE_LOCATION,
                                Manifest.permission.ACCESS_COARSE_LOCATION
                        }, 1001);
            }
        }
    }

    public void toast(String message, int duration) {
        Context ctx = getContext();
        if (ctx != null) {
            Toast.makeText(ctx, message, duration).show();
        }
    }

    // Method: Pass bluetooth status to other fragments
    private void sendBluetoothStatus(String msg) {
        if (!isAdded()) return;
        updateDeviceName();
        Log.d(TAG, "sending device name, " + msg);
        Context ctx = getContext();
        if (ctx == null) return;
        Intent intent = new Intent("getConnectedDevice");
        intent.putExtra("name", msg);
        LocalBroadcastManager.getInstance(ctx).sendBroadcast(intent);
    }

    // Method: pass received text to messages fragment
    private void sendReceived(String msg) {
        if (!isAdded()) {
            Log.w(TAG, "sendReceived() called when fragment not added; dropping msg");
            return;
        }
        Log.d(TAG, "received msg: " + msg);
        Context ctx = getContext();
        if (ctx == null) return;
        Intent intent = new Intent("getReceived");
        intent.putExtra("received", msg);
        LocalBroadcastManager.getInstance(ctx).sendBroadcast(intent);
    }

    private void updateDeviceName() {
        if (!deviceSingleton.getDeviceName().equals("")) {
            connectedDevice = deviceSingleton.getDeviceName();
        }
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        // stop receiving messages to this (now-detached) UI
        bController.setHandler(null);
        mHandler.removeCallbacksAndMessages(null);

        // ensure discovery is stopped to avoid leaking the view
        if (bAdapter != null && bAdapter.isDiscovering()) bAdapter.cancelDiscovery();

        // extra safety — receivers are primarily unregistered in onStop
        unregisterReceivers();

        binding = null;
    }
}
