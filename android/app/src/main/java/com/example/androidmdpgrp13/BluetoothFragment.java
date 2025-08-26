package com.example.androidmdpgrp13;

import android.annotation.SuppressLint;
import android.Manifest;
import android.bluetooth.*;
import android.content.*;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.view.*;
import android.widget.Toast;

import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.annotation.*;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;
import androidx.recyclerview.widget.*;

import com.google.android.material.button.MaterialButton;
import com.google.android.material.materialswitch.MaterialSwitch;

import java.io.IOException;
import java.util.*;

public class BluetoothFragment extends Fragment {

    private static final UUID SPP_UUID =
            UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    private BluetoothAdapter btAdapter;
    private BluetoothSocket socket;

    // UI (new IDs from frag_bluetooth.xml)
    private MaterialSwitch switchBluetooth;
    private MaterialButton btnScan;
    private RecyclerView rvDiscovered, rvPaired;

    // Data
    private final List<BluetoothDevice> discoveredDevices = new ArrayList<>();
    private final List<String> discoveredLines = new ArrayList<>();
    private SimpleStringAdapter discoveredAdapter;

    // Launchers
    private ActivityResultLauncher<String[]> permLauncher;
    private ActivityResultLauncher<Intent> enableBtLauncher;

    private boolean isSPlus() { return Build.VERSION.SDK_INT >= 31; }
    private boolean hasScanPermission() {
        if (!isSPlus()) return true;
        return ContextCompat.checkSelfPermission(requireContext(), Manifest.permission.BLUETOOTH_SCAN)
                == PackageManager.PERMISSION_GRANTED;
    }
    private boolean hasConnectPermission() {
        if (!isSPlus()) return true;
        return ContextCompat.checkSelfPermission(requireContext(), Manifest.permission.BLUETOOTH_CONNECT)
                == PackageManager.PERMISSION_GRANTED;
    }
    private boolean hasLegacyLocationForDiscovery() {
        if (isSPlus()) return true;
        return ContextCompat.checkSelfPermission(requireContext(), Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED;
    }
    private boolean canScan()    { return isSPlus() ? hasScanPermission()    : hasLegacyLocationForDiscovery(); }
    private boolean canConnect() { return hasConnectPermission(); }

    // ---- Receiver: show each find immediately ----
    private final BroadcastReceiver receiver = new BroadcastReceiver() {
        @Override public void onReceive(Context ctx, Intent intent) {
            String action = intent.getAction();

            if (BluetoothAdapter.ACTION_DISCOVERY_STARTED.equals(action)) {
                toast("Scanning…");

            } else if (BluetoothDevice.ACTION_FOUND.equals(action)) {
                BluetoothDevice d = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if (d == null) return;

                String addr = safeGetAddress(d);
                if (addr != null && !contains(discoveredDevices, addr)) {
                    discoveredDevices.add(d);
                    String name = safeGetName(d);
                    String line = (name != null ? name : "Unknown Device") + "\n" + addr;

                    discoveredLines.add(line);
                    discoveredAdapter.notifyItemInserted(discoveredLines.size() - 1);

                    toast("Found: " + line);
                    android.util.Log.d("BluetoothFragment", "Discovered: " + line);
                }

            } else if (BluetoothAdapter.ACTION_DISCOVERY_FINISHED.equals(action)) {
                toast("Discovery finished (" + discoveredDevices.size() + ")");
            }
        }
    };

    private void toast(String s) { if (isAdded()) Toast.makeText(requireContext(), s, Toast.LENGTH_SHORT).show(); }
    private boolean contains(List<BluetoothDevice> list, String addr) {
        for (BluetoothDevice d : list) { String a = safeGetAddress(d); if (addr.equals(a)) return true; }
        return false;
    }
    private @Nullable String safeGetName(BluetoothDevice d) {
        try { if (!isSPlus() || hasConnectPermission()) return d.getName(); } catch (SecurityException ignored) {}
        return null;
    }
    private @Nullable String safeGetAddress(BluetoothDevice d) {
        try { if (!isSPlus() || hasConnectPermission()) return d.getAddress(); } catch (SecurityException ignored) {}
        return null;
    }

    @Override public void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        permLauncher = registerForActivityResult(
                new ActivityResultContracts.RequestMultiplePermissions(), r -> {});
        enableBtLauncher = registerForActivityResult(
                new ActivityResultContracts.StartActivityForResult(), r -> {});
    }

    @Nullable
    @Override public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup parent, @Nullable Bundle b) {
        return inflater.inflate(R.layout.frag_bluetooth, parent, false);
    }

    @Override
    public void onViewCreated(@NonNull View v, @Nullable Bundle b) {
        super.onViewCreated(v, b);

        // Bind new views
        switchBluetooth = v.findViewById(R.id.switchBluetooth);
        btnScan         = v.findViewById(R.id.btnScan);
        rvDiscovered    = v.findViewById(R.id.rvDiscovered);
        rvPaired        = v.findViewById(R.id.rvPaired);

        // Bluetooth adapter
        BluetoothManager bm = (BluetoothManager) requireContext().getSystemService(Context.BLUETOOTH_SERVICE);
        btAdapter = bm != null ? bm.getAdapter() : BluetoothAdapter.getDefaultAdapter();
        if (btAdapter == null) {
            toast("Bluetooth not supported on this device");
            btnScan.setEnabled(false);
            switchBluetooth.setEnabled(false);
            return;
        }

        // Recycler setup (left list for discovered)
        discoveredAdapter = new SimpleStringAdapter(discoveredLines, pos -> {
            if (pos >= 0 && pos < discoveredDevices.size()) connect(discoveredDevices.get(pos));
        });
        rvDiscovered.setLayoutManager(new LinearLayoutManager(requireContext()));
        rvDiscovered.setAdapter(discoveredAdapter);

        // (Optional) Right list can later show bonded devices
        rvPaired.setLayoutManager(new LinearLayoutManager(requireContext()));
        rvPaired.setAdapter(new SimpleStringAdapter(Collections.emptyList(), null));

        // Switch = enable/disable Bluetooth UI behavior
        switchBluetooth.setChecked(btAdapter.isEnabled());
        switchBluetooth.setOnCheckedChangeListener((button, checked) -> {
            if (checked) ensureBtOn(); else { stopScan(); toast("Bluetooth switch off (no adapter toggle)"); }
        });

        btnScan.setOnClickListener(v1 -> startScan());

        ensurePermissions();
    }

    @Override public void onStart() {
        super.onStart();
        IntentFilter f = new IntentFilter();
        f.addAction(BluetoothAdapter.ACTION_DISCOVERY_STARTED);
        f.addAction(BluetoothDevice.ACTION_FOUND);
        f.addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
        requireContext().registerReceiver(receiver, f);
    }

    @Override public void onStop() {
        super.onStop();
        try { requireContext().unregisterReceiver(receiver); } catch (Exception ignored) {}
        if (btAdapter != null) safeCancelDiscovery();
        closeSocketQuietly();
    }

    // ---- Permissions/Enable ----
    private void ensurePermissions() {
        if (isSPlus()) {
            List<String> need = new ArrayList<>();
            if (!hasScanPermission())    need.add(Manifest.permission.BLUETOOTH_SCAN);
            if (!hasConnectPermission()) need.add(Manifest.permission.BLUETOOTH_CONNECT);
            if (!need.isEmpty()) permLauncher.launch(need.toArray(new String[0]));
        } else if (!hasLegacyLocationForDiscovery()) {
            permLauncher.launch(new String[]{ Manifest.permission.ACCESS_FINE_LOCATION });
        }
    }

    private void ensureBtOn() {
        if (btAdapter != null && !btAdapter.isEnabled()) {
            Intent i = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            enableBtLauncher.launch(i);
        } else {
            toast("Bluetooth already ON");
        }
    }

    // ---- Discovery ----
    private void startScan() {
        if (btAdapter == null) return;
        if (!canScan()) { ensurePermissions(); return; }

        discoveredDevices.clear();
        discoveredLines.clear();
        discoveredAdapter.notifyDataSetChanged();

        safeCancelDiscovery();
        toast("Starting scan…");
        safeStartDiscovery();
    }

    private void stopScan() { safeCancelDiscovery(); }

    @SuppressLint("MissingPermission")
    private void safeStartDiscovery() {
        try { btAdapter.startDiscovery(); } catch (SecurityException ignored) {}
    }

    @SuppressLint("MissingPermission")
    private void safeCancelDiscovery() {
        try { btAdapter.cancelDiscovery(); } catch (SecurityException ignored) {}
    }

    // ---- Connect ----
    private void connect(BluetoothDevice device) {
        if (btAdapter == null) return;
        if (!canConnect()) { ensurePermissions(); return; }

        String n = safeGetName(device);
        String a = safeGetAddress(device);
        final String label = (n != null) ? n : (a != null ? a : "device");
        final BluetoothDevice target = device;

        toast("Connecting to " + label);

        new Thread(() -> {
            try {
                safeCancelDiscovery();
                @SuppressLint("MissingPermission")
                BluetoothSocket s = target.createRfcommSocketToServiceRecord(SPP_UUID);
                s.connect();
                socket = s;
                requireActivity().runOnUiThread(() -> toast("Connected to " + label));
            } catch (IOException | SecurityException e) {
                requireActivity().runOnUiThread(() -> toast("Connection failed: " + e.getMessage()));
                closeSocketQuietly();
            }
        }).start();
    }

    private void closeSocketQuietly() {
        if (socket != null) { try { socket.close(); } catch (Exception ignored) {} socket = null; }
    }

    // ---- Minimal adapter for the RecyclerViews ----
    private static class SimpleStringAdapter extends RecyclerView.Adapter<SimpleStringAdapter.VH> {
        interface OnItemClick { void onClick(int position); }
        private final List<String> items;
        private final OnItemClick click;

        SimpleStringAdapter(List<String> items, OnItemClick click) {
            this.items = items;
            this.click = click;
        }

        static class VH extends RecyclerView.ViewHolder {
            final android.widget.TextView tv;
            VH(View itemView) { super(itemView); tv = itemView.findViewById(android.R.id.text1); }
        }

        @NonNull @Override public VH onCreateViewHolder(@NonNull ViewGroup p, int vType) {
            View v = LayoutInflater.from(p.getContext())
                    .inflate(android.R.layout.simple_list_item_1, p, false);
            return new VH(v);
        }

        @Override public void onBindViewHolder(@NonNull VH h, int pos) {
            h.tv.setText(items.get(pos));
            h.itemView.setOnClickListener(_v -> { if (click != null) click.onClick(h.getAdapterPosition()); });
        }

        @Override public int getItemCount() { return items.size(); }
    }
}
