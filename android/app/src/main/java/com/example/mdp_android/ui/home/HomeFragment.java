package com.example.mdp_android.ui.home;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.ImageButton;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import androidx.annotation.Nullable;
import androidx.appcompat.widget.SwitchCompat;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.Observer;
import androidx.lifecycle.ViewModelProvider;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;
import androidx.recyclerview.widget.GridLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.example.mdp_android.R;
import com.example.mdp_android.controllers.DeviceSingleton;
import com.example.mdp_android.controllers.RpiController;
import com.example.mdp_android.databinding.FragmentHomeBinding;
import com.example.mdp_android.ui.CustomSpinnerAdapter;
import com.example.mdp_android.ui.grid.Map;
import com.google.android.material.switchmaterial.SwitchMaterial;

import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class HomeFragment extends Fragment {
    private static final String TAG = "HomeFragment";
    private static final int SPAN = 2;

    private FragmentHomeBinding binding;
    private HomeViewModel homeViewModel;
    private String connectedDevice = "";
    DeviceSingleton deviceSingleton;

    public Map map;
    private ImageButton up, down, left, right;
    private Button resetBtn, startBtn, loadPresetBtn;
    private TextView robotStatus, targetStatus, bluetoothTextView, obsData, status;
    private RecyclerView obsList;
    private static RecyclerAdapter obsItems;
    private RecyclerView.LayoutManager layoutManager;
    private SwitchMaterial setRobot, setDirection;
    private ToggleButton setTaskType;
    private Spinner spinnerLoadPreset;
    private boolean isSpinnerTouched = false;
    private boolean suppressSwitchCallbacks = false;
    private Toast currentToast;

    // Robot defaults
    private int presetRobotX = 5;
    private int presetRobotY = 5;
    private String presetRobotDirection = "N";

    // --- New: fields for the custom switch using your existing IDs ---
    private SwitchCompat taskTypeGroup;   // ID: taskTypeGroup  (SwitchCompat in XML)
    private TextView btnImageRec;         // ID: btn_image_rec  (TextView over the left half)
    private TextView btnFastestCar;       // ID: btn_fastest_car (TextView over the right half)

    // Helper to style labels (selected side dark text, the other side white)
    private void styleTaskLabels(boolean fastest, TextView labelIR, TextView labelFC) {
        int white = ContextCompat.getColor(labelIR.getContext(), R.color.seg_white);
        int black = ContextCompat.getColor(labelIR.getContext(), R.color.black);
        if (fastest) {
            labelIR.setTextColor(white); labelIR.setAlpha(0.85f);
            labelFC.setTextColor(black); labelFC.setAlpha(1f);
        } else {
            labelIR.setTextColor(black); labelIR.setAlpha(1f);
            labelFC.setTextColor(white); labelFC.setAlpha(0.85f);
        }
    }


    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        homeViewModel = new ViewModelProvider(this).get(HomeViewModel.class);
        binding = FragmentHomeBinding.inflate(inflater, container, false);
        View root = binding.getRoot();

        root.setBackgroundResource(R.drawable.background_pattern);
        bluetoothTextView = binding.textViewBluetooth;

        homeViewModel.setStatus("Ready to Start");

        // Receivers
        LocalBroadcastManager.getInstance(requireActivity())
                .registerReceiver(mNameReceiver, new IntentFilter("getConnectedDevice"));
        LocalBroadcastManager.getInstance(requireActivity())
                .registerReceiver(mTextReceiver, new IntentFilter("getReceived"));
        LocalBroadcastManager.getInstance(requireActivity())
                .registerReceiver(initialStatusReceiver, new IntentFilter("getStatus"));

        return root;
    }

    // Connection name receiver
    private final BroadcastReceiver mNameReceiver = new BroadcastReceiver() {
        @Override public void onReceive(Context context, Intent intent) {
            String deviceName = intent.getStringExtra("name");
            if (deviceName == null || deviceName.equals("")) {
                connectedDevice = "";
            } else {
                connectedDevice = deviceName;
                Log.d(TAG, "onReceive: -msg- " + connectedDevice);
            }
            if (deviceSingleton == null) deviceSingleton = DeviceSingleton.getInstance();
            deviceSingleton.setDeviceName(connectedDevice);
            updateBluetoothStatus();
        }
    };

    // Messages receiver
    private final BroadcastReceiver mTextReceiver = new BroadcastReceiver() {
        @Override public void onReceive(Context context, Intent intent) {
            Log.d(TAG, "receiving messages");
            String textReceived = intent.getStringExtra("received");
            log("TEXT RECEIVED IS: " + textReceived);
            JSONObject response = RpiController.readRpiMessages(textReceived);
            String messageType = RpiController.getRpiMessageType(textReceived);
            log("MESSAGE TYPE IS:" + messageType);

            String statusStr;

            if (textReceived != null && textReceived.startsWith("ROBOT")) {
                toast("C10 SCENARIO DETECTED");
                String[] parts = textReceived.split(",");
                JSONObject responseC10 = new JSONObject();
                if (parts.length == 4) {
                    try {
                        int x = Integer.parseInt(parts[1].trim());
                        int y = Integer.parseInt(parts[2].trim());
                        String direction = parts[3].trim();
                        responseC10.put("x", x);
                        responseC10.put("y", y);
                        responseC10.put("dir", direction);
                    } catch (Exception e) { log("Invalid number format / json put: " + e); }
                }
                updateRobotPosition(responseC10);
                statusStr = RpiController.getRobotStatus(responseC10);
                homeViewModel.setRobotStatus(statusStr);

            } else if ("path".equals(messageType)) {
                Log.d(TAG, "JUST PATH");
                try {
                    homeViewModel.setStatus("Looking for target");
                    ArrayList<ArrayList<Integer>> path = RpiController.getPath(response);
                    map.setExploredPath(path);
                    map.animateRobotPath(path);
                    Log.d(TAG, "path: " + path.get(0));
                } catch (Exception e) { log("empty path received: " + e); }

            } else if ("robot".equals(messageType)) {
                Log.d(TAG, "JUST ROBOT");
                statusStr = RpiController.getRobotStatus(response);
                homeViewModel.setRobotStatus(statusStr);
                updateRobotPosition(response);

            } else if ("image".equals(messageType)) {
                int count = textReceived != null ? textReceived.length() : 0;
                Log.e(TAG, "Number of characters: " + count);
                if (count > 67) {
                    Log.d(TAG, "JUST BOTH IMAGE AND PATH");
                    statusStr = RpiController.getTargetStatus(response);
                    updateObstacle(response);
                    try {
                        Map.Obstacle o = map.getObstacle(Integer.parseInt(response.getString("obs_id")));
                        if (o != null) {
                            int x = o.getObsXCoor() - 1;
                            int y = o.getObsYCoor() - 1;
                            statusStr = statusStr + " at (" + x + ", " + y + ") facing " + o.getDirection();
                        } else {
                            statusStr = "Invalid ID received";
                            toast("Invalid Obstacle ID received");
                        }
                    } catch (Exception e) { log("Failed to parse JSON: " + e); }
                    homeViewModel.setTargetStatus(statusStr);
                    homeViewModel.setStatus("Target detected");

                    int firstIndex = textReceived.indexOf('{');
                    int secondIndex = textReceived.indexOf('{', firstIndex + 1);
                    int thirdIndex = textReceived.indexOf('{', secondIndex + 1);
                    String json2String = textReceived.substring(thirdIndex);
                    JSONObject response2 = RpiController.readSecondJSONMessages(json2String);
                    try {
                        homeViewModel.setStatus("Looking for target");
                        ArrayList<ArrayList<Integer>> path = RpiController.getPath(response2);
                        map.setExploredPath(path);
                        map.animateRobotPath(path);
                        Log.d(TAG, "path: " + path.get(0));
                    } catch (Exception e) { log("empty path received: " + e); }

                } else {
                    Log.d(TAG, "JUST IMAGE");
                    statusStr = RpiController.getTargetStatus(response);
                    updateObstacle(response);
                    try {
                        Map.Obstacle o = map.getObstacle(Integer.parseInt(response.getString("obs_id")));
                        if (o != null) {
                            int x = o.getObsXCoor() - 1;
                            int y = o.getObsYCoor() - 1;
                            statusStr = statusStr + " at (" + x + ", " + y + ") facing " + o.getDirection();
                        } else {
                            statusStr = "Invalid ID received";
                            toast("Invalid Obstacle ID received");
                        }
                    } catch (Exception e) { log("Failed to parse JSON: " + e); }
                    homeViewModel.setTargetStatus(statusStr);
                    homeViewModel.setStatus("Target detected");
                }
            }
        }
    };

    private final BroadcastReceiver initialStatusReceiver = new BroadcastReceiver() {
        @Override public void onReceive(Context context, Intent intent) {
            String textReceived = intent.getStringExtra("robot");
            homeViewModel.setRobotStatus(textReceived);
        }
    };

    @Override public void onDestroyView() {
        super.onDestroyView();
        binding = null;
    }

    @Override public void onDestroy() {
        super.onDestroy();
        LocalBroadcastManager.getInstance(requireActivity()).unregisterReceiver(mNameReceiver);
        LocalBroadcastManager.getInstance(requireActivity()).unregisterReceiver(mTextReceiver);
        LocalBroadcastManager.getInstance(requireActivity()).unregisterReceiver(initialStatusReceiver);
    }

    @Override
    public void onViewCreated(View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        // Robot control UI
        up = view.findViewById(R.id.imageButton_up);
        down = view.findViewById(R.id.imageButton_down);
        left = view.findViewById(R.id.imageButton_left);
        right = view.findViewById(R.id.imageButton_right);
        resetBtn = view.findViewById(R.id.button_reset);
        robotStatus = view.findViewById(R.id.textView_robotStatus);
        targetStatus = view.findViewById(R.id.textView_targetCoor);
        status = view.findViewById(R.id.textView_Status);
        obsData = view.findViewById(R.id.textView_obsData);
        obsList = view.findViewById(R.id.recyclerView_obsList);
        map = view.findViewById(R.id.mapView);
        setRobot = view.findViewById(R.id.button_startpoint);
        setDirection = view.findViewById(R.id.button_setDirection);

        // Tap-to-edit obstacles only when both toggles are OFF
        map.setObstacleTapToEditEnabled(!setDirection.isChecked() && !setRobot.isChecked());

        // Open edit dialog on obstacle tap (only when both toggles are OFF)
        map.setOnObstacleTapListener(obstacle -> {
            if (setDirection.isChecked() || setRobot.isChecked()) return;
            AddObstacleDialogFragment f = new AddObstacleDialogFragment();
            Bundle args = new Bundle();
            args.putBoolean("EDIT_MODE", true);
            args.putInt(AddObstacleDialogFragment.B_ID,  obstacle.getObsID());
            args.putInt(AddObstacleDialogFragment.B_X,   obstacle.getObsXCoor() - 1);
            args.putInt(AddObstacleDialogFragment.B_Y,   obstacle.getObsYCoor() - 1);
            args.putString(AddObstacleDialogFragment.B_DIR, obstacle.getDirection());
            f.setArguments(args);
            f.show(getParentFragmentManager(), "AddObstacleDialog");
        });

        startBtn = view.findViewById(R.id.button_start);
        setTaskType = view.findViewById(R.id.button_taskType);

        // Add Obstacle dialog launcher
        Button addObstacleBtn = view.findViewById(R.id.button_add_obstacle);
        if (addObstacleBtn != null) {
            addObstacleBtn.setOnClickListener(v -> {
                ArrayList<Integer> used = new ArrayList<>(map.getPlacedObstacleIds());
                AddObstacleDialogFragment f = new AddObstacleDialogFragment();
                Bundle args = new Bundle();
                args.putIntegerArrayList("USED_IDS", used);
                f.setArguments(args);
                f.show(getParentFragmentManager(), "AddObstacleDialog");
            });
        }

        // Dialog result handler
        getParentFragmentManager().setFragmentResultListener(
                AddObstacleDialogFragment.REQUEST_KEY,
                getViewLifecycleOwner(),
                (requestKey, bundle) -> {
                    if (map == null) return;
                    int id  = bundle.getInt(AddObstacleDialogFragment.B_ID);
                    int x   = bundle.getInt(AddObstacleDialogFragment.B_X);
                    int y   = bundle.getInt(AddObstacleDialogFragment.B_Y);
                    String dirLabel = bundle.getString(AddObstacleDialogFragment.B_DIR, "Up");

                    boolean idExists = map.getPlacedObstacleIds().contains(id);
                    boolean ok;
                    if (idExists) {
                        ok = map.updateObstacleById(id, x, y, dirLabel);
                        toast(ok ? ("Updated obstacle " + id + " → (" + x + ", " + y + "), " + dirLabel)
                                : "Can't update (occupied or out of bounds).");
                    } else {
                        ok = map.placeObstacleFromDialog(id, x, y, dirLabel);
                        if (ok) {
                            modifyObstacleVisibility(id - 1, false);
                            toast("Added obstacle " + id + " at (" + x + ", " + y + "), " + dirLabel);
                        } else {
                            toast("Can't add (occupied or out of bounds).");
                        }
                    }
                }
        );

        // ---- Task type switch (StackOverflow style; using your existing IDs) ----
        taskTypeGroup = view.findViewById(R.id.taskTypeGroup);      // SwitchCompat in XML
        btnImageRec   = view.findViewById(R.id.btn_image_rec);      // left label
        btnFastestCar = view.findViewById(R.id.btn_fastest_car);    // right label

        // Initial sync with legacy ToggleButton: checked => Fastest Car
        boolean fastestNow = (setTaskType != null && setTaskType.isChecked());
        taskTypeGroup.setChecked(fastestNow);
        styleTaskLabels(fastestNow, btnImageRec, btnFastestCar);

        // Switch -> legacy toggle
        taskTypeGroup.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (setTaskType != null && setTaskType.isChecked() != isChecked) {
                setTaskType.setChecked(isChecked);
            }
            styleTaskLabels(isChecked, btnImageRec, btnFastestCar);
        });

        // Legacy toggle -> switch (if other code flips it)
        if (setTaskType != null) {
            setTaskType.setOnCheckedChangeListener((btn, b) -> {
                if (taskTypeGroup.isChecked() != b) taskTypeGroup.setChecked(b);
                styleTaskLabels(b, btnImageRec, btnFastestCar);
            });
        }

        // -------------------------------------------------------------------

        setupPresetSpinner(view);
        createObstacleList();

        startBtn.setOnClickListener(v -> {
            map.sendMapToRpi();
            map.setStart(true);
            homeViewModel.setStatus("Looking for target");
            toast("Start Task: " + map.getTaskType());
        });

        resetBtn.setOnClickListener(v -> {
            map.clearGrid();
            obsItems.setAllVisibility(true);
            map.setStart(false);
        });

        up.setOnClickListener(v -> {
            toast("move forward");
            ArrayList<String> commands = new ArrayList<>();
            commands.add("SF050");
            RpiController.sendToRpi(RpiController.getNavDetails(commands));
        });

        down.setOnClickListener(v -> {
            toast("move backwards");
            ArrayList<String> commands = new ArrayList<>();
            commands.add("SB050");
            RpiController.sendToRpi(RpiController.getNavDetails(commands));
        });

        left.setOnClickListener(v -> {
            toast("turn left");
            ArrayList<String> commands = new ArrayList<>();
            commands.add("LF090");
            RpiController.sendToRpi(RpiController.getNavDetails(commands));
        });

        right.setOnClickListener(v -> {
            toast("turn right");
            ArrayList<String> commands = new ArrayList<>();
            commands.add("RF090");
            RpiController.sendToRpi(RpiController.getNavDetails(commands));
        });

        // Set Robot
        setRobot.setOnCheckedChangeListener((buttonView, isChecked) -> {
            toast(isChecked ? "Select a cell to place robot" : "Cancel");
            map.setCanDrawRobot(isChecked);
            map.setObstacleTapToEditEnabled(!isChecked && !setDirection.isChecked());
            if (suppressSwitchCallbacks) return;
            if (isChecked && setDirection.isChecked()) {
                suppressSwitchCallbacks = true;
                setDirection.setChecked(false);
                suppressSwitchCallbacks = false;
                map.setCanSetDirection(false);
            }
        });

        // Set Direction
        setDirection.setOnCheckedChangeListener((buttonView, isChecked) -> {
            toast(isChecked ? "Select object to change direction" : "Cancel");
            map.setCanSetDirection(isChecked);
            map.setObstacleTapToEditEnabled(!isChecked && !setRobot.isChecked());
            if (suppressSwitchCallbacks) return;
            if (isChecked && setRobot.isChecked()) {
                suppressSwitchCallbacks = true;
                setRobot.setChecked(false);
                suppressSwitchCallbacks = false;
                map.setCanDrawRobot(false);
            }
        });

        // Legacy ToggleButton listener (kept) – now just updates map
        setTaskType.setOnCheckedChangeListener((compoundButton, b) -> {
            toast(b ? "Task type: Fastest Car" : "Task type: Image Recognition");
            map.setTaskType(b);
            // keep SwitchCompat in sync if needed (already synced above)
            if (taskTypeGroup != null && taskTypeGroup.isChecked() != b) {
                taskTypeGroup.setChecked(b);
            }
        });
    }

    // --- ViewModel bindings & lifecycle ---
    @Override
    public void onResume() {
        super.onResume();

        bluetoothTextView.setText(homeViewModel.getReceivedText().getValue());
        updateObstacleListVisibility();
        homeViewModel.getReceivedText().observe(getViewLifecycleOwner(), s -> bluetoothTextView.setText(s));

        robotStatus.setText(homeViewModel.getRobotStatus().getValue());
        homeViewModel.getRobotStatus().observe(getViewLifecycleOwner(), s -> robotStatus.setText(s));

        targetStatus.setText(homeViewModel.getTargetStatus().getValue());
        homeViewModel.getTargetStatus().observe(getViewLifecycleOwner(), s -> targetStatus.setText(s));

        status.setText(homeViewModel.getStatus().getValue());
        homeViewModel.getStatus().observe(getViewLifecycleOwner(), s -> status.setText(s));
    }

    public void createObstacleList() {
        obsItems = new RecyclerAdapter(new String[]{"1","2","3","4","5","6","7","8","9","10"});
        obsList.setAdapter(obsItems);
        layoutManager = new GridLayoutManager(getContext(), SPAN);
        obsList.setLayoutManager(layoutManager);
    }

    public void updateBluetoothStatus() {
        log("updating bluetooth status in home fragment...");
        deviceSingleton = DeviceSingleton.getInstance();

        if (!deviceSingleton.getDeviceName().equals("")) {
            connectedDevice = deviceSingleton.getDeviceName();
            homeViewModel.setReceivedText(getContext().getString(R.string.bluetooth_device_connected) + connectedDevice);
            homeViewModel.setStatus("Ready to start");
        } else {
            homeViewModel.setReceivedText(getContext().getString(R.string.bluetooth_device_connected_not));
        }
    }

    public void updateRobotPosition(JSONObject robot) {
        try {
            int x = Integer.parseInt(robot.getString("x"));
            int y = Integer.parseInt(robot.getString("y"));
            String d = robot.getString("dir");
            if (map.isWithinCanvasRegion(x + 1, y + 1)) {
                map.setRobotCoor(x + 1, y + 1, d);
            } else {
                toast("Invalid coordinates received");
            }
        } catch (Exception e) { log("Failed to parse JSON: " + e); }
    }

    public void updateObstacle(JSONObject target) {
        try {
            toast("Image detected!");
            log("Target is" + target);
            int obsID = Integer.parseInt(target.getString("obs_id"));
            log("OBS ID IS " + obsID);
            int imgID = Integer.parseInt(target.getString("img_id"));
            if (imgID == 0) {
                toast("Cannot detect image");
            } else {
                map.setObsTargetID(obsID, imgID);
            }
        } catch (Exception e) { log("Failed to parse JSON: " + e); }
    }

    public static void modifyObstacleVisibility(int position, boolean visible) {
        obsItems.setItemVisibility(position, visible);
        Log.d(TAG, "set obstacle " + position + " to " + visible);
    }

    public void log(String message) { Log.d(TAG, message); }

    public void toast(String message) {
        if (currentToast != null) currentToast.cancel();
        currentToast = Toast.makeText(binding.getRoot().getContext(), message, Toast.LENGTH_SHORT);
        currentToast.show();
    }

    /** Updates the visibility of the obstacle list based on the current map state */
    public void updateObstacleListVisibility() {
        if (map == null || obsItems == null) return;
        List<Integer> placedObstacleIds = map.getPlacedObstacleIds();
        for (int i = 0; i < obsItems.getItemCount(); i++) {
            boolean isVisible = !placedObstacleIds.contains(i + 1);
            obsItems.setItemVisibility(i, isVisible);
        }
        obsItems.notifyDataSetChanged();
    }

    public void setupPresetSpinner(View view) {
        spinnerLoadPreset = view.findViewById(R.id.spinner_load_preset); // initialize the preset spinner
        // Create a list of items for the spinner.
        spinnerLoadPreset = binding.spinnerLoadPreset;
        List<String> spinnerData = Arrays.asList("Load Preset", "Preset 1", "Preset 2", "Preset 3", "Preset 4", "Preset 5", "Preset 6");
        CustomSpinnerAdapter adapter = new CustomSpinnerAdapter(requireContext(), spinnerData);
        spinnerLoadPreset.setAdapter(adapter);

        spinnerLoadPreset.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                isSpinnerTouched = true;
                return false;
            }
        });

        spinnerLoadPreset.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                if (!isSpinnerTouched) return;

                // Code to execute when an item is selected
                String selectedOption = parent.getItemAtPosition(position).toString();
//                toast("Selected: " + selectedOption);

                // clear the map grid
                map.clearGrid();
                obsItems.setAllVisibility(true);
                map.setStart(false);

                presetRobotDirection = "E";
                switch (selectedOption) {
                    case "Preset 1":
                        presetRobotX = 2;
                        presetRobotY = 2;
                        presetRobotDirection = "E";

                        map.setRobotCoor(presetRobotX, presetRobotY, presetRobotDirection); // set the robot's position and draw the robot on the map
                        map.setPresetObstacles(selectedOption); // draw the preset obstacles on the map

                        modifyObstacleVisibility(0, false); // clear obstacle 1 from the recycler view
                        modifyObstacleVisibility(1, false);
                        modifyObstacleVisibility(2, false);
                        modifyObstacleVisibility(3, false);
                        modifyObstacleVisibility(4, false);
                        modifyObstacleVisibility(5, false);
//                        modifyObstacleVisibility(6, false);
//                        modifyObstacleVisibility(7, false);

                        toast("Preset 1 has been loaded!");
                        break;

                    case "Preset 2":
                        presetRobotX = 2;
                        presetRobotY = 2;
                        presetRobotDirection = "N";
                        map.setRobotCoor(presetRobotX, presetRobotY, presetRobotDirection);
                        map.setPresetObstacles(selectedOption);

                        modifyObstacleVisibility(0, false); // clear obstacle 1 from the recycler view
                        modifyObstacleVisibility(1, false);
                        modifyObstacleVisibility(2, false);
                        modifyObstacleVisibility(3, false);
                        modifyObstacleVisibility(4, false);
                        modifyObstacleVisibility(5, false);
                        modifyObstacleVisibility(6, false);
                        modifyObstacleVisibility(7, false);
                        modifyObstacleVisibility(8, false);

                        toast("Preset 2 has been loaded!");
                        break;

                    case "Preset 3":
                        presetRobotX = 2;
                        presetRobotY = 2;
                        presetRobotDirection = "E";
                        map.setRobotCoor(presetRobotX, presetRobotY, presetRobotDirection);
                        map.setPresetObstacles(selectedOption);

                        for (int i = 0; i < obsItems.getItemCount(); i++) {
                            modifyObstacleVisibility(i, false);   // hides every ID that the preset just placed
                        }

                        toast("Preset 3 has been loaded!");
                        break;

                    case "Preset 4":
                        presetRobotX = 2;
                        presetRobotY = 2;
                        presetRobotDirection = "N";
                        map.setRobotCoor(presetRobotX, presetRobotY, presetRobotDirection);
                        map.setPresetObstacles(selectedOption);

                        for (int i = 0; i < obsItems.getItemCount(); i++) {
                            modifyObstacleVisibility(i, false);   // hides every ID that the preset just placed
                        }

                        toast("Preset 4 has been loaded!");
                        break;

                    case "Preset 5":
                        presetRobotX = 2;
                        presetRobotY = 2;
                        presetRobotDirection = "E";
                        map.setRobotCoor(presetRobotX, presetRobotY, presetRobotDirection);
                        map.setPresetObstacles(selectedOption);

                        for (int i = 0; i < obsItems.getItemCount(); i++) {
                            modifyObstacleVisibility(i, false);   // hides every ID that the preset just placed
                        }

                        toast("Preset 5 has been loaded!");
                        break;

                    case "Preset 6":
                        presetRobotX = 2;
                        presetRobotY = 2;
                        presetRobotDirection = "E";
                        map.setRobotCoor(presetRobotX, presetRobotY, presetRobotDirection);
                        map.setPresetObstacles(selectedOption);

                        for (int i = 0; i < obsItems.getItemCount(); i++) {
                            modifyObstacleVisibility(i, false);   // hides every ID that the preset just placed
                        }

                        toast("Preset 6 has been loaded!");
                        break;
                }

                // Reset the flag after handling selection
                isSpinnerTouched = false;
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                // Code to execute when nothing is selected
            }
        });
    }
}
