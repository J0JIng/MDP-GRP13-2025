package com.example.mdp_android.controllers;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.util.Log;
import android.os.Handler;
import android.os.Message;
import android.os.Bundle;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import com.example.mdp_android.Constants;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.UUID;

public class BluetoothController {
    // ================== Config ==================
    private static final String TAG = "BluetoothController";
    public static final String DEVICE_NAME = "MDP_DEATH";
    private static final UUID MY_UUID =
            UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private static final int BUFFER_SIZE = 2048;

    // ================== Android BT ==================
    private final BluetoothAdapter mAdapter;
    private AcceptThread mAcceptThread;
    private ConnectThread mConnectThread;
    private ConnectedThread mConnectedThread;

    // for LocalBroadcasts
    private Context appContext;

    // ================== State / UI ==================
    private int mState;
    private int mNewState;
    private Handler mHandler; // handler that gets info from Bluetooth service

    // Messages to UI (keep your original values)
    public interface MessageConstants {
        int MESSAGE_STATE_CHANGE = 1;
        int MESSAGE_READ = 2;
        int MESSAGE_WRITE = 3;
        int MESSAGE_DEVICE_NAME = 4;
        int MESSAGE_TOAST = 5;
        int MESSAGE_PICTURE = 6;
    }

    // Connection states (keep your original values)
    public interface StateConstants {
        int STATE_NONE = 0;
        int STATE_LISTEN = 1;
        int STATE_CONNECTING = 2;
        int STATE_CONNECTED = 3;
        int STATE_DISCONNECTED = 4;
    }

    // ================== Ctors / setters ==================
    // Legacy ctor (no broadcasts)
    public BluetoothController(Handler handler) {
        mAdapter = BluetoothAdapter.getDefaultAdapter();
        mState = StateConstants.STATE_NONE;
        mNewState = mState;
        mHandler = handler;
        this.appContext = null;
    }

    // Preferred ctor (enables broadcasts)
    public BluetoothController(Context context, Handler handler) {
        mAdapter = BluetoothAdapter.getDefaultAdapter();
        mState = StateConstants.STATE_NONE;
        mNewState = mState;
        mHandler = handler;
        this.appContext = context != null ? context.getApplicationContext() : null;
    }

    public void setHandler(Handler handler) { this.mHandler = handler; }

    public void setAppContext(Context context) {
        this.appContext = context != null ? context.getApplicationContext() : null;
    }

    public synchronized int getState() { return mState; }

    private synchronized void updateUIBluetoothStatus() {
        mState = getState();
        Log.d(TAG, "Bluetooth Status: " + mNewState + " -> " + mState);
        mNewState = mState;
        if (mHandler != null) {
            mHandler.obtainMessage(MessageConstants.MESSAGE_STATE_CHANGE, mNewState, -1)
                    .sendToTarget();
        } else {
            Log.w(TAG, "Handler is null in updateUIBluetoothStatus()");
        }
    }

    private void toast(String text) {
        if (mHandler == null) return;
        Message msg = mHandler.obtainMessage(MessageConstants.MESSAGE_TOAST);
        Bundle bundle = new Bundle();
        bundle.putString("toast", text);
        msg.setData(bundle);
        msg.sendToTarget();
    }

    // ================== Public control ==================
    public synchronized void start() {
        Log.d(TAG, "start listening for devices");

        if (mConnectThread != null) { mConnectThread.cancel(); mConnectThread = null; }
        if (mConnectedThread != null) { mConnectedThread.cancel(); mConnectedThread = null; }

        if (mAcceptThread == null) {
            mAcceptThread = new AcceptThread();
            mAcceptThread.start();
        }

        updateUIBluetoothStatus();
    }

    public synchronized void connect(BluetoothDevice device) {
        Log.d(TAG, "connect to: " + device);

        if (mState == StateConstants.STATE_CONNECTING) {
            if (mConnectThread != null) { mConnectThread.cancel(); mConnectThread = null; }
        }
        if (mConnectedThread != null) { mConnectedThread.cancel(); mConnectedThread = null; }

        mConnectThread = new ConnectThread(device);
        mConnectThread.start();

        updateUIBluetoothStatus();
    }

    public synchronized void connected(BluetoothSocket socket, BluetoothDevice device) {
        Log.d(TAG, "connected to " + device);

        if (mConnectThread != null) { mConnectThread.cancel(); mConnectThread = null; }
        if (mConnectedThread != null) { mConnectedThread.cancel(); mConnectedThread = null; }
        if (mAcceptThread != null) { mAcceptThread.cancel(); mAcceptThread = null; }

        mConnectedThread = new ConnectedThread(socket);
        mConnectedThread.start();

        // Send the device name to UI (guard Android 12+ permission)
        if (mHandler != null) {
            try {
                Message msg = mHandler.obtainMessage(MessageConstants.MESSAGE_DEVICE_NAME);
                Bundle bundle = new Bundle();
                bundle.putString("device_name", device != null ? device.getName() : "Unknown");
                msg.setData(bundle);
                mHandler.sendMessage(msg);
            } catch (SecurityException se) {
                Log.w(TAG, "getName() requires BLUETOOTH_CONNECT on Android 12+. Skipping name.", se);
            }
        }

        updateUIBluetoothStatus();
    }

    public synchronized void stop() {
        Log.d(TAG, "stop");

        if (mConnectThread != null) { mConnectThread.cancel(); mConnectThread = null; }
        if (mConnectedThread != null) { mConnectedThread.cancel(); mConnectedThread = null; }
        if (mAcceptThread != null) { mAcceptThread.cancel(); mAcceptThread = null; }

        mState = StateConstants.STATE_NONE;
        updateUIBluetoothStatus();
    }

    public void write(byte[] out) {
        ConnectedThread r;
        synchronized (this) {
            if (mState != StateConstants.STATE_CONNECTED) {
                Log.w(TAG, "write() ignored: not connected");
                return;
            }
            r = mConnectedThread;
        }
        if (r != null) r.write(out);
    }

    // ================== Failure handling ==================
    private void connectionFailed() {
        Log.d(TAG, "connection failed");
        toast("Unable to connect device");
        mState = StateConstants.STATE_NONE;
        updateUIBluetoothStatus();
        Log.d(TAG, "restarting listening mode...");
        BluetoothController.this.start();
    }

    private void connectionLost() {
        Log.d(TAG, "connection lost");
        toast("Device connection was lost");
        mState = StateConstants.STATE_DISCONNECTED;
        updateUIBluetoothStatus();
        BluetoothController.this.start();
    }

    private void safeClose(BluetoothSocket s) {
        if (s != null) { try { s.close(); } catch (IOException ignored) {} }
    }

    // ================== Threads ==================
    /** Server-side listener */
    private class AcceptThread extends Thread {
        private BluetoothServerSocket mmServerSocket;

        public AcceptThread() {
            try {
                mmServerSocket = mAdapter.listenUsingRfcommWithServiceRecord(DEVICE_NAME, MY_UUID);
                Log.d(TAG, "mmServerSocket: " + mmServerSocket);
            } catch (IOException e) {
                Log.e(TAG, "listenUsingRfcommWithServiceRecord failed", e);
            }
            mState = StateConstants.STATE_LISTEN;
        }

        public void run() {
            Log.d(TAG, "BEGIN mAcceptThread " + this);
            BluetoothSocket socket = null;
            try {
                if (mmServerSocket != null) socket = mmServerSocket.accept();
            } catch (IOException e) {
                Log.e(TAG, "Socket's accept() failed", e);
            }

            if (socket != null) {
                Log.d(TAG, "connection accepted");
                synchronized (BluetoothController.this) {
                    switch (mState) {
                        case StateConstants.STATE_LISTEN:
                        case StateConstants.STATE_CONNECTING:
                            connected(socket, socket.getRemoteDevice());
                            break;
                        case StateConstants.STATE_NONE:
                        case StateConstants.STATE_CONNECTED:
                            safeClose(socket);
                            break;
                    }
                }
            }
            Log.i(TAG, "END mAcceptThread");
            cancel();
        }

        public void cancel() {
            Log.d(TAG, "Close server socket");
            try {
                if (mmServerSocket != null) mmServerSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "Could not close server socket", e);
            }
            mmServerSocket = null;
        }
    }

    /** Client-side: outgoing connection with robust fallbacks */
    private class ConnectThread extends Thread {
        private BluetoothSocket mmSocket;
        private final BluetoothDevice mmDevice;

        public ConnectThread(BluetoothDevice device) {
            mmDevice = device;
            try {
                mmSocket = device.createRfcommSocketToServiceRecord(MY_UUID); // secure first
                Log.d(TAG, "client secure socket prepared: " + mmSocket);
            } catch (IOException e) {
                Log.e(TAG, "createRfcommSocketToServiceRecord failed", e);
            }
            mState = StateConstants.STATE_CONNECTING;
        }

        public void run() {
            Log.i(TAG, "BEGIN mConnectThread");
            if (mAdapter != null) mAdapter.cancelDiscovery();

            // Attempt #1: secure SPP
            try {
                if (mmSocket != null) {
                    mmSocket.connect();
                    Log.d(TAG, "connected (secure): " + mmSocket);
                    synchronized (BluetoothController.this) { mConnectThread = null; }
                    connected(mmSocket, mmDevice);
                    return;
                }
            } catch (IOException secureErr) {
                Log.w(TAG, "Secure connect failed, trying fallbacks", secureErr);
                safeClose(mmSocket);
                mmSocket = null;
            }

            // Attempt #2: insecure SPP
            try {
                if (mAdapter != null) mAdapter.cancelDiscovery();
                BluetoothSocket insecure =
                        mmDevice.createInsecureRfcommSocketToServiceRecord(MY_UUID);
                insecure.connect();
                Log.d(TAG, "connected (insecure)");
                synchronized (BluetoothController.this) { mConnectThread = null; }
                connected(insecure, mmDevice);
                return;
            } catch (IOException insecureErr) {
                Log.w(TAG, "Insecure connect failed, trying channel-1", insecureErr);
            }

            // Attempt #3: reflection channel 1 (HC-05/Windows friendly)
            try {
                if (mAdapter != null) mAdapter.cancelDiscovery();
                java.lang.reflect.Method m =
                        mmDevice.getClass().getMethod("createInsecureRfcommSocket", int.class);
                BluetoothSocket ch1 = (BluetoothSocket) m.invoke(mmDevice, 1);
                ch1.connect();
                Log.d(TAG, "connected (channel 1)");
                synchronized (BluetoothController.this) { mConnectThread = null; }
                connected(ch1, mmDevice);
            } catch (Exception ch1Err) {
                Log.e(TAG, "All connect attempts failed", ch1Err);
                connectionFailed();
            }
        }

        public void cancel() { safeClose(mmSocket); }
    }

    /** Active connection: read/write data safely */
    private class ConnectedThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;
        private final byte[] buffer = new byte[BUFFER_SIZE];

        public ConnectedThread(BluetoothSocket socket) {
            Log.d(TAG, "create connected thread");
            mmSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;
            try { tmpIn = socket.getInputStream(); }
            catch (IOException e) { Log.e(TAG, "create input stream failed", e); }
            try { tmpOut = socket.getOutputStream(); }
            catch (IOException e) { Log.e(TAG, "create output stream failed", e); }
            mmInStream = tmpIn;
            mmOutStream = tmpOut;
            mState = StateConstants.STATE_CONNECTED;
        }

        public void run() {
            Log.d(TAG, "BEGIN mConnectedThread");

            while (mState == StateConstants.STATE_CONNECTED) {
                try {
                    if (mmInStream == null) {
                        Log.w(TAG, "InputStream is null; treating as lost");
                        connectionLost();
                        break;
                    }

                    int numBytes = mmInStream.read(buffer);

                    // read() returns -1 on end-of-stream; 0 is unusual—treat as lost or continue.
                    if (numBytes <= 0) {
                        Log.w(TAG, "read() returned " + numBytes + "; treating as lost");
                        connectionLost();
                        break;
                    }

                    // Copy the exact payload for UI & logs
                    byte[] payload = new byte[numBytes];
                    System.arraycopy(buffer, 0, payload, 0, numBytes);

                    // Post to UI
                    if (mHandler != null) {
                        Message readMsg = mHandler.obtainMessage(
                                MessageConstants.MESSAGE_READ, numBytes, -1, payload);
                        readMsg.sendToTarget();
                    }

                    // Log & (safe) repository write
                    String receivedMessage = new String(payload, StandardCharsets.UTF_8);
                    Log.d(TAG, "bluetooth received message");
                    Log.d(TAG, "RECEIVED: " + receivedMessage);
                    try {
                        MessageRepository.getInstance()
                                .addReceivedMessage(getCurrentTime() + " " + receivedMessage);
                    } catch (Throwable t) {
                        Log.w(TAG, "MessageRepository unavailable; skipping addReceivedMessage", t);
                    }

                    // NEW: notify app via LocalBroadcast so UI updates instantly
                    if (appContext != null) {
                        Intent i = new Intent(Constants.ACTION_BLUETOOTH_MESSAGE_RECEIVED);
                        i.putExtra(Constants.EXTRA_BLUETOOTH_MESSAGE, receivedMessage);
                        LocalBroadcastManager.getInstance(appContext).sendBroadcast(i);
                        Log.d(TAG, "Broadcast out with appContext");
                    }

                } catch (IOException e) {
                    Log.e(TAG, "Input stream disconnected", e);
                    connectionLost();
                    break;
                }
            }
            Log.i(TAG, "END mConnectedThread");
        }

        public void write(byte[] bytes) {
            try {
                if (mmOutStream == null) {
                    Log.w(TAG, "write() skipped: output stream is null");
                    return;
                }
                mmOutStream.write(bytes);
                mmOutStream.flush();

                // Echo back what we actually sent
                if (mHandler != null) {
                    Message writtenMsg = mHandler.obtainMessage(
                            MessageConstants.MESSAGE_WRITE, -1, -1, bytes);
                    writtenMsg.sendToTarget();
                }

                String sentMessage = new String(bytes, StandardCharsets.UTF_8);
                try {
                    MessageRepository.getInstance()
                            .addSentMessage(getCurrentTime() + " " + sentMessage);
                } catch (Throwable t) {
                    Log.w(TAG, "MessageRepository unavailable; skipping addSentMessage", t);
                }
                Log.d(TAG, "SENT: " + sentMessage);

            } catch (IOException e) {
                Log.e(TAG, "Error sending data", e);
            }
        }

        public void cancel() {
            try { mmSocket.close(); } catch (IOException e) {
                Log.e(TAG, "Could not close the connect socket", e);
            }
        }

        private String getCurrentTime() {
            return new SimpleDateFormat("dd/MM/yyyy HH:mm:ss").format(new Date());
        }
    }
}
