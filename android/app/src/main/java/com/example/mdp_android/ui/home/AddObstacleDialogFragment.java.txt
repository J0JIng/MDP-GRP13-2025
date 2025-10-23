package com.example.mdp_android.ui.home;

import android.app.Dialog;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.NumberPicker;
import android.widget.Spinner;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AlertDialog;
import androidx.fragment.app.DialogFragment;

import com.example.mdp_android.R;

public class AddObstacleDialogFragment extends DialogFragment {

    public static final String REQUEST_KEY = "add_obstacle_result";
    public static final String B_ID  = "id";
    public static final String B_X   = "x";
    public static final String B_Y   = "y";
    public static final String B_DIR = "dir";

    // NEW: flag for edit mode (set by caller)
    public static final String ARG_EDIT_MODE = "EDIT_MODE";

    @NonNull @Override
    public Dialog onCreateDialog(@Nullable Bundle savedInstanceState) {
        View view = LayoutInflater.from(requireContext())
                .inflate(R.layout.dialog_add_obstacle, null, false);

        NumberPicker id = view.findViewById(R.id.picker_id);
        NumberPicker px = view.findViewById(R.id.picker_x);
        NumberPicker py = view.findViewById(R.id.picker_y);
        Spinner dir = view.findViewById(R.id.spinner_dir);

        id.setMinValue(1);  id.setMaxValue(10);
        px.setMinValue(0);  px.setMaxValue(19);
        py.setMinValue(0);  py.setMaxValue(19);

        ArrayAdapter<CharSequence> a = ArrayAdapter.createFromResource(
                requireContext(), R.array.directions_display, android.R.layout.simple_spinner_item);
        a.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        dir.setAdapter(a);

        // --- Prefill if in EDIT mode ---
        Bundle args = getArguments();
        boolean editMode = args != null && args.getBoolean(ARG_EDIT_MODE, false);
        if (editMode && args != null) {
            int idVal = args.getInt(B_ID, 1);
            int xVal  = args.getInt(B_X, 0);
            int yVal  = args.getInt(B_Y, 0);
            String dirRaw = args.getString(B_DIR, "Up"); // could be "N/E/S/W" or "Up/Right/Down/Left"

            id.setValue(idVal);
            px.setValue(xVal);
            py.setValue(yVal);

            String label = toDisplayLabel(dirRaw); // normalize to "Up/Right/Down/Left"
            int sel = findPositionIgnoreCase(a, label);
            if (sel >= 0) dir.setSelection(sel, false);
        }

        return new AlertDialog.Builder(requireContext())
                .setTitle(editMode ? "Edit Obstacle" : "Add Obstacle")
                .setView(view)
                .setPositiveButton(editMode ? "Update" : "Add", (d, which) -> {
                    Bundle b = new Bundle();
                    b.putInt(B_ID,  id.getValue());
                    b.putInt(B_X,   px.getValue());
                    b.putInt(B_Y,   py.getValue());
                    b.putString(B_DIR, dir.getSelectedItem().toString()); // "Up/Right/Down/Left"
                    getParentFragmentManager().setFragmentResult(REQUEST_KEY, b);
                })
                .setNegativeButton("Cancel", (d, which) -> d.dismiss())
                .create();
    }

    // Map "N/E/S/W" or already-display labels to "Up/Right/Down/Left"
    private String toDisplayLabel(String raw) {
        if (raw == null) return "Up";
        String r = raw.trim();
        if (equalsAnyIgnoreCase(r, "N", "North", "Up"))    return "Up";
        if (equalsAnyIgnoreCase(r, "E", "East",  "Right")) return "Right";
        if (equalsAnyIgnoreCase(r, "S", "South", "Down"))  return "Down";
        if (equalsAnyIgnoreCase(r, "W", "West",  "Left"))  return "Left";
        return "Up";
    }

    private boolean equalsAnyIgnoreCase(String s, String... opts) {
        for (String o : opts) if (s.equalsIgnoreCase(o)) return true;
        return false;
    }

    private int findPositionIgnoreCase(ArrayAdapter<CharSequence> adapter, String value) {
        for (int i = 0; i < adapter.getCount(); i++) {
            CharSequence item = adapter.getItem(i);
            if (item != null && item.toString().equalsIgnoreCase(value)) return i;
        }
        return -1;
    }
}
