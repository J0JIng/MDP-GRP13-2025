package com.example.mdp_android.ui.messages;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ClipData;
import android.content.ClipboardManager;
import android.database.DataSetObserver;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ListView;
import android.widget.Button;
import android.widget.Toast;

import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import com.example.mdp_android.R;
import com.example.mdp_android.Constants;
import com.example.mdp_android.controllers.BluetoothControllerSingleton;
import com.example.mdp_android.controllers.DeviceSingleton;
import com.example.mdp_android.controllers.MessageRepository;
import com.example.mdp_android.databinding.FragmentMessagesBinding;
import com.example.mdp_android.controllers.RpiController;

import org.json.JSONObject;

import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Collections;
import java.util.Date;
import java.util.List;

public class MessagesFragment extends Fragment {
    private static String TAG = "MessagesFragment";
    private FragmentMessagesBinding binding;
    private MessagesViewModel messagesViewModel;
    private Button sendBtn;
    private ImageButton copyReceivedBtn, copySentBtn;
    private ImageButton clearReceivedBtn, clearSentBtn;
    private EditText eMessage;
    private ListView lvSentMessages;
    private ListView lvReceivedMessages;
    private static ArrayAdapter<String> aSentMessages;
    private static ArrayAdapter<String> aReceivedMessages;

    public static com.example.mdp_android.controllers.BluetoothController bController;

    private boolean receiverRegistered = false;

    DeviceSingleton deviceSingleton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        messagesViewModel = new ViewModelProvider(this).get(MessagesViewModel.class);

        binding = FragmentMessagesBinding.inflate(inflater, container, false);
        View root = binding.getRoot();

        root.setBackgroundResource(R.drawable.background_pattern);

        // Build controller with appContext so LocalBroadcasts work
        bController = BluetoothControllerSingleton.getInstance(
                requireContext().getApplicationContext(),
                new Handler(Looper.getMainLooper())
        );

        lvSentMessages = root.findViewById(R.id.listView_sent);
        lvReceivedMessages = root.findViewById(R.id.listview_received);

        eMessage = root.findViewById(R.id.editText_sendMessage);
        sendBtn = root.findViewById(R.id.button_send);
        copyReceivedBtn = root.findViewById(R.id.imageButton_copyReceived);
        clearReceivedBtn = root.findViewById(R.id.imageButton_clearReceived);
        copySentBtn = root.findViewById(R.id.imageButton_copySent);
        clearSentBtn = root.findViewById(R.id.imageButton_clearSent);

        deviceSingleton = DeviceSingleton.getInstance();

        if (aSentMessages == null || aReceivedMessages == null) {
            aSentMessages = new ArrayAdapter<>(binding.getRoot().getContext(), R.layout.message_item);
            aReceivedMessages = new ArrayAdapter<>(binding.getRoot().getContext(), R.layout.message_item);
        }

        lvSentMessages.setAdapter(aSentMessages);
        aSentMessages.registerDataSetObserver(new DataSetObserver() {
            @Override public void onChanged() {
                super.onChanged();
                lvSentMessages.setSelection(aSentMessages.getCount() - 1);
            }
        });

        lvReceivedMessages.setAdapter(aReceivedMessages);
        aReceivedMessages.registerDataSetObserver(new DataSetObserver() {
            @Override public void onChanged() {
                super.onChanged();
                lvReceivedMessages.setSelection(aReceivedMessages.getCount() - 1);
            }
        });

        sendBtn.setOnClickListener(new View.OnClickListener() {
            @Override public void onClick(View view) {
                String message = eMessage.getText().toString();
                if (deviceSingleton.getDeviceName().equals("")) {
                    toast("Bluetooth not connected to any device");
                } else if (message.equals("")) {
                    toast("Hey, don't just send empty strings");
                } else {
                    sendMessage(message);
                    toast("message sent: " + message);
                    eMessage.setText("");
                    appendASentMessages(message);
                }
            }
        });

        clearReceivedBtn.setOnClickListener(new View.OnClickListener() {
            @Override public void onClick(View view) {
                aReceivedMessages.clear();
                toast("messages cleared");
            }
        });

        clearSentBtn.setOnClickListener(new View.OnClickListener() {
            @Override public void onClick(View view) {
                aSentMessages.clear();
                toast("messages cleared");
            }
        });

        copyReceivedBtn.setOnClickListener(new View.OnClickListener() {
            @Override public void onClick(View view) {
                ClipboardManager clipboard = (ClipboardManager) getActivity().getSystemService(Context.CLIPBOARD_SERVICE);
                StringBuilder sb = new StringBuilder();
                for (int i = 0; i < aReceivedMessages.getCount(); i++) {
                    sb.append(aReceivedMessages.getItem(i)).append("\n");
                }
                String allReceivedMessages = sb.toString().trim();
                clipboard.setPrimaryClip(ClipData.newPlainText("received_messages", allReceivedMessages));
                toast("Copied to clipboard!");
            }
        });

        copySentBtn.setOnClickListener(new View.OnClickListener() {
            @Override public void onClick(View view) {
                ClipboardManager clipboard = (ClipboardManager) getActivity().getSystemService(Context.CLIPBOARD_SERVICE);
                StringBuilder sb = new StringBuilder();
                for (int i = 0; i < aSentMessages.getCount(); i++) {
                    sb.append(aSentMessages.getItem(i)).append("\n");
                }
                String allSentMessages = sb.toString().trim();
                clipboard.setPrimaryClip(ClipData.newPlainText("received_messages", allSentMessages));
                toast("Copied to clipboard!");
            }
        });

        return root;
    }

    @Override
    public void onStart() {
        super.onStart();
        // Register receiver only while this tab is visible
        if (!receiverRegistered && getContext() != null) {
            LocalBroadcastManager.getInstance(getContext())
                    .registerReceiver(mTextReceiver,
                            new IntentFilter(Constants.ACTION_BLUETOOTH_MESSAGE_RECEIVED));
            receiverRegistered = true;
        }
    }

    @Override
    public void onStop() {
        super.onStop();
        // Unregister to avoid duplicates and leaks
        if (receiverRegistered && getContext() != null) {
            LocalBroadcastManager.getInstance(getContext()).unregisterReceiver(mTextReceiver);
            receiverRegistered = false;
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        // refresh on return (still keep for safety)
        updateReceivedMessages();
        updateSentMessages();
    }

    private void updateReceivedMessages() {
        List<String> messages = MessageRepository.getInstance().getReceivedMessages(); // copy
        // Repo stores newest-first; show newest at bottom
        Collections.reverse(messages);
        if (aReceivedMessages != null) {
            aReceivedMessages.clear();
            aReceivedMessages.addAll(messages);
            aReceivedMessages.notifyDataSetChanged();
        }
    }

    private void updateSentMessages() {
        List<String> messages = MessageRepository.getInstance().getSentMessages(); // copy
        Collections.reverse(messages);
        if (aSentMessages != null) {
            aSentMessages.clear();
            aSentMessages.addAll(messages);
            aSentMessages.notifyDataSetChanged();
        }
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        binding = null;
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        // onStop already unregisters; keep this as a safety no-op
        if (receiverRegistered && getContext() != null) {
            try {
                LocalBroadcastManager.getInstance(getContext()).unregisterReceiver(mTextReceiver);
            } catch (IllegalArgumentException ignored) {}
            receiverRegistered = false;
        }
    }

    private final BroadcastReceiver mTextReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            Log.d(TAG, "receiving messages (broadcast)");
            String textReceived = intent.getStringExtra(Constants.EXTRA_BLUETOOTH_MESSAGE);

            // Optional parse (do not append directly to adapter to avoid duplicates)
            try { RpiController.readRpiMessages(textReceived); } catch (Throwable ignored) {}

            // Refresh from repository (controller already saved the message there)
            View root = getView();
            if (root != null) {
                root.post(() -> updateReceivedMessages());
            } else {
                updateReceivedMessages();
            }
        }
    };

    public static String getCurrentTime() {
        SimpleDateFormat formatter = new SimpleDateFormat("dd/MM/yyyy HH:mm:ss");
        Date date = new Date();
        return formatter.format(date);
    }

    public static void appendASentMessages(String message) {
        // Append to END so observer scrolls to newest
        aSentMessages.add(getCurrentTime() + ": " + message);
        aSentMessages.notifyDataSetChanged();
    }

    // Kept for compatibility; broadcast path uses updateReceivedMessages()
    public static void appendAReceivedMessages(String message) {
        aReceivedMessages.add(getCurrentTime() + ": " + message);
        aReceivedMessages.notifyDataSetChanged();
    }

    private void sendMessage(String m) {
        if (bController != null) {
            bController.write(m.getBytes(StandardCharsets.UTF_8));
        } else {
            toast("Bluetooth controller not ready");
        }
    }

    public void toast(String message) {
        Toast.makeText(getContext(), message, Toast.LENGTH_SHORT).show();
    }
}
