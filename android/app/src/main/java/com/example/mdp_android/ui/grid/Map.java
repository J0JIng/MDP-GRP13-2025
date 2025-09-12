package com.example.mdp_android.ui.grid;

import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.util.Log;
import android.view.DragEvent;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;
import android.widget.Toast;
import android.os.Handler;


import androidx.annotation.Nullable;

import androidx.core.content.ContextCompat;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import com.example.mdp_android.R;
import com.example.mdp_android.controllers.RpiController;
import com.example.mdp_android.ui.home.HomeFragment;
import com.example.mdp_android.ui.home.HomeViewModel;

import java.util.ArrayList;
import java.util.List;
//import java.util.logging.Handler;




public class Map extends View {
    // ➊ Tap-to-edit API
    public interface OnObstacleTapListener {
        void onObstacleTapped(Obstacle obstacle);
    }

    private boolean tapToEditEnabled = false;
    private OnObstacleTapListener obstacleTapListener;

    public void setObstacleTapToEditEnabled(boolean enabled) { this.tapToEditEnabled = enabled; }
    public void setOnObstacleTapListener(OnObstacleTapListener l) { this.obstacleTapListener = l; }

    // ➋ Hit-test caches (view-space)
    private final java.util.List<android.graphics.RectF> obstacleHitRects = new java.util.ArrayList<>();
    private final java.util.List<Obstacle> obstacleHitList = new java.util.ArrayList<>();

    // Helper: register an obstacle's drawn rect for taps
    private void registerObstacleHitRect(android.graphics.Canvas canvas,
                                         Obstacle o,
                                         float left, float top, float right, float bottom) {
        android.graphics.RectF viewRect = new android.graphics.RectF(left, top, right, bottom);
        // Convert from the scaled/panned canvas to view coordinates so MotionEvent coords match.
        canvas.getMatrix().mapRect(viewRect);
        obstacleHitRects.add(viewRect);
        obstacleHitList.add(o);
    }
    private ScaleGestureDetector mScaleDetector;

    // Tap detection without breaking drag
    private int touchSlop;
    private float downX, downY;
    private int downObstacleIndex = -1;
    private boolean potentialTap = false;


    private float mScaleFactor = 1.f;
    private float focusx, focusy;
    private boolean pan = false;

    private static final String TAG = "MapController";
    private static final int NUM_COLS = 20, NUM_ROWS = 20;
    private static final float WALL_THICKNESS = 5;
    private static final String DEFAULT_DIRECTION = "N";
    private static boolean mapDrawn = false;
    private static Cell[][] cells;
    private static float cellSize;
    private Paint obstacleFacePaint, obstaclePaint, robotPaint, exploredPaint, cellPaint, linePaint, whitePaint, targetTextPaint, imageIdentifiedPaint, imageIdentifiedFacePaint, gridPaint;

    private static int currentSelected = -1;
    private static ArrayList<Obstacle> obstacleCoor = new ArrayList<>();

    // robot start coordinates
    private static Robot robot = new Robot();
    private Bitmap robotDirectionBitmap;
    private static boolean canDrawRobot = false;
    private static boolean canSetDirection = false;
    private static boolean start = false;

    private static boolean taskType = false;

    public void setStart(boolean start) {
        this.start = start;
    }

    public void setCanSetDirection(boolean setDir) {
        canSetDirection = setDir;
    }

    public void setCanDrawRobot(boolean draw) {
        canDrawRobot = draw;
    }

    public void setTaskType(boolean task) {
        taskType = task;
    }

    public String getTaskType() {
        if (taskType) return "FASTEST_PATH";
        else return "EXPLORATION";
    }

    // initialize map
    public Map(Context context, @Nullable AttributeSet attributes) {
        super(context, attributes);

        // create objects
        obstacleFacePaint = new Paint();
        obstaclePaint = new Paint();
        robotPaint = new Paint();
        exploredPaint = new Paint();
        cellPaint = new Paint();
        linePaint = new Paint();
        whitePaint = new Paint();
        targetTextPaint = new Paint();
        imageIdentifiedPaint = new Paint();
        imageIdentifiedFacePaint = new Paint();
        gridPaint = new Paint();

        // paint for paths
        exploredPaint.setColor(Color.parseColor("#A4FEFF"));

        // paint for grid cells
        cellPaint.setColor(ContextCompat.getColor(context, R.color.map));
        linePaint.setColor(ContextCompat.getColor(context, R.color.background));
        linePaint.setStrokeWidth(WALL_THICKNESS);
        linePaint.setStyle(Paint.Style.FILL_AND_STROKE);

        //paint for grid numbers
        gridPaint.setColor(ContextCompat.getColor(context, R.color.gridnumbers));
        gridPaint.setStyle(Paint.Style.FILL_AND_STROKE);
        gridPaint.setTextSize(20);

        // paint for robot
        robotPaint.setColor(ContextCompat.getColor(context, R.color.pink_500));

        // paint for obstacle
        obstaclePaint.setColor(Color.BLACK);
        obstacleFacePaint.setColor(Color.RED);
        whitePaint.setColor(Color.WHITE);
        whitePaint.setStyle(Paint.Style.FILL_AND_STROKE);
        whitePaint.setTextSize(20);

        // paint for images
        targetTextPaint.setColor(Color.WHITE);
        targetTextPaint.setStyle(Paint.Style.FILL_AND_STROKE);
        targetTextPaint.setFakeBoldText(true);
        targetTextPaint.setTextSize(24);
        targetTextPaint.setTextAlign(Paint.Align.CENTER);
        imageIdentifiedPaint.setColor(Color.BLACK);  // This line can help for task C7, the image recognition thing, when the face turns yellow upon being recognized by the robot
        imageIdentifiedFacePaint.setColor(Color.parseColor("#FFFF00"));
        mScaleDetector = new ScaleGestureDetector(context, new ScaleListener());
        touchSlop = android.view.ViewConfiguration.get(context).getScaledTouchSlop();
    }

    // Grid Cell object
    private class Cell {
        float sX, sY, eX, eY;
        String type;
        String id = "-1";
        int obsIndex = -1;
        Paint paint;

        public Cell(float startX, float startY, float endX, float endY, String type, Paint paint) {
            this.sX = startX;
            this.sY = startY;
            this.eX = endX;
            this.eY = endY;
            this.type = type;
            this.paint = paint;

        }

        public String getId() {
            return id;
        }

        public void setId(String id) {
            this.id = id;
        }

        public String getType() {
            return this.type;
        }

        public int getObsIndex() {
            return obsIndex;
        }

        public void setObsIndex(int index) {
            this.obsIndex = index;
        }

        public void setType(String type) {
            this.type = type;
            switch (type) {
                case "image":
                    this.paint = imageIdentifiedPaint; // This line is important for task C7
                    break;
                case "obstacle":
                    this.paint = obstaclePaint;
                    break;
                case "robot":
                    this.paint = robotPaint;
                    break;
                case "unexplored":
                    this.paint = cellPaint;
                    break;
                case "explored":
                    this.paint = exploredPaint;
                    break;
            }
        }
    }

    // obstacle object
    public class Obstacle {
        // coordinates of position
        public int x, y, obsID, targetID;
        public String direction = DEFAULT_DIRECTION;

        public Obstacle(int x, int y) {
            this.x = x;
            this.y = y;
        }

        public Obstacle(int x, int y, int obsID) {
            this.x = x;
            this.y = y;
            this.obsID = obsID;
            this.targetID = -1;
        }

        public int getObsID() {
            return this.obsID;
        }

        public int getObsXCoor() {
            return this.x;
        }

        public void setObsXCoor(int x) {
            this.x = x;
        }

        public int getObsYCoor() {
            return this.y;
        }

        public void setObsYCoor(int y) {
            this.y = y;
        }

        public int getTargetID() {
            return this.targetID;
        }

        public void setTargetID(int targetID) {
            this.targetID = targetID;
        }

        public String getDirection() {
            return this.direction;
        }

        public void setDirection(String d) {
            this.direction = d;
        }

    }

    public static class Robot {
        public int x, y;
        public String direction = "N";

        public Robot() {
            Log.d(TAG, "creating robot");
            this.x = -1;
            this.y = -1;
        }

        public Robot(int x, int y) {
            this.x = x;
            this.y = y;
        }

        public int getX() {
            return this.x;
        }

        public int getY() {
            return this.y;
        }

        public String getDirection() {
            return this.direction;
        }

        public void setX(int x) {
            this.x = x;
        }

        public void setY(int y) {
            this.y = y;
        }

        public void setDirection(String d) {
            this.direction = d;
        }
    }

    private void createCell() {
        log("create cells");
        cells = new Cell[NUM_COLS + 1][NUM_ROWS + 1];
        this.cellSize = calculateCellSize();

        for (int x = 0; x <= NUM_COLS; x++)
            for (int y = 0; y <= NUM_ROWS; y++)
                cells[x][y] = new Cell(
                        x * cellSize + (cellSize / 30),
                        y * cellSize + (cellSize / 30),
                        (x + 1) * cellSize,
                        (y + 1) * cellSize,
                        "unexplored",
                        cellPaint);
    }

    private float calculateCellSize() {
        return (getWidth() / (NUM_COLS + 1));
    }

    @Override
    protected void onDraw(Canvas canvas) {
        log("start drawing map");
        super.onDraw(canvas);

        obstacleHitRects.clear();
        obstacleHitList.clear();

        canvas.save();
        canvas.scale(mScaleFactor, mScaleFactor, focusx, focusy);
        if (!mapDrawn) {
            this.createCell();
            mapDrawn = true;
        }

        if (NUM_COLS == 0 || NUM_ROWS == 0) return;
        drawGrids(canvas);
        drawGridNumbers(canvas);
        drawObstacle(canvas);
        if (robot.getX() != -1 && robot.getY() != -1) drawRobot(canvas);
        drawIdentifiedImage(canvas);
        canvas.restore();
        log("map drawn successfully");
    }

    private class ScaleListener extends ScaleGestureDetector.SimpleOnScaleGestureListener {
        @Override
        public boolean onScaleBegin(ScaleGestureDetector detector) {
            focusx = detector.getFocusX();
            focusy = detector.getFocusY();
            pan = true;
            return super.onScaleBegin(detector);
        }

        @Override
        public boolean onScale(ScaleGestureDetector detector) {
            mScaleFactor *= detector.getScaleFactor();

            // Don't let the object get too small or too large.
            mScaleFactor = Math.max(0.1f, Math.min(mScaleFactor, 5.0f));

            invalidate();
            return true;
        }
    }

    private void drawGrids(Canvas canvas) {
        for (int x = 1; x <= NUM_COLS; x++)
            for (int y = 0; y < NUM_ROWS; y++) {
                canvas.drawRect(cells[x][y].sX, cells[x][y].sY, cells[x][y].eX, cells[x][y].eY, cells[x][y].paint);
            }

        // draw vertical lines
        for (int c = 0; c <= NUM_COLS; c++) {
            canvas.drawLine(cells[c][0].sX - (cellSize / 30) + cellSize, cells[c][0].sY - (cellSize / 30),
                    cells[c][0].sX - (cellSize / 30) + cellSize, cells[c][NUM_ROWS - 1].eY + (cellSize / 30), linePaint);
        }

        // draw horizontal lines
        for (int r = 0; r <= NUM_ROWS; r++) {
            canvas.drawLine(
                    cells[1][r].sX, cells[1][r].sY - (cellSize / 30),
                    cells[NUM_COLS][r].eX, cells[NUM_COLS][r].sY - (cellSize / 30), linePaint);
        }
    }

    private void drawGridNumbers(Canvas canvas) {
        for (int x = 1; x <= NUM_COLS; x++) {
            if (x > 9)
                canvas.drawText(Integer.toString(x - 1), cells[x][NUM_ROWS].sX + (cellSize / 5), cells[x][NUM_ROWS].sY + (cellSize / 2), gridPaint);
            else
                canvas.drawText(Integer.toString(x - 1), cells[x][NUM_ROWS].sX + (cellSize / 3), cells[x][NUM_ROWS].sY + (cellSize / 2), gridPaint);
        }
        for (int y = 0; y < NUM_ROWS; y++) {
            if ((this.convertRow(y)) > 10)
                canvas.drawText(Integer.toString(19 - y), cells[0][y].sX + (cellSize / 4), cells[0][y].sY + (cellSize / 1.5f), gridPaint);
            else
                canvas.drawText(Integer.toString(19 - y), cells[0][y].sX + (cellSize / 2f), cells[0][y].sY + (cellSize / 1.5f), gridPaint);
        }
    }

    private void drawObstacle(Canvas canvas) {
        log("drawing obstacles on map");
        RectF rect = null;
        if (obstacleCoor.size() > 0) {
            for (int i = 0; i < obstacleCoor.size(); i++) {
                int col = obstacleCoor.get(i).getObsXCoor();
                int row = this.convertRow(obstacleCoor.get(i).getObsYCoor());
                int obsID = obstacleCoor.get(i).getObsID();
                String direction = obstacleCoor.get(i).getDirection();

                rect = new RectF(col * cellSize, row * cellSize, (col + 1) * cellSize, (row + 1) * cellSize);
                canvas.drawRect(rect, obstaclePaint);
                canvas.drawText(obsID + "", col * cellSize + cellSize / 2.5f, row * cellSize + cellSize / 1.5f, whitePaint);
                // draw direction
                drawDirection(canvas, col, row, direction, obstacleFacePaint);

                // NEW: register this obstacle’s on-screen hit area for tap detection
                registerObstacleHitRect(canvas, obstacleCoor.get(i),
                        rect.left, rect.top, rect.right, rect.bottom);
            }
        }
    }


    /**
     * Gets the list of placed obstacles in the map
     * (related to maintaining persistence in the recycler view when switching tab bars!)
     * @return
     */
    public List<Integer> getPlacedObstacleIds() {
        List<Integer> placedIds = new ArrayList<>();
        for (Obstacle obstacle : obstacleCoor) { // Assuming obstacleCoor is accessible
            placedIds.add(obstacle.getObsID());
        }
        return placedIds;
    }

    private void drawDirection(Canvas canvas, int col, int row, String direction, Paint color) {
        float left = col * cellSize;
        float top = row * cellSize;
        float right = (col + 1) * cellSize;
        float bottom = (row + 1) * cellSize;
        float dWidth = 0.1f;
        switch (direction) {
            case "N":
                canvas.drawRect(left, top, right, (row + dWidth) * cellSize, color);
                break;
            case "S":
                canvas.drawRect(left, (row + 1 - dWidth) * cellSize, right, bottom, color);
                break;
            case "E":
                canvas.drawRect((col + 1 - dWidth) * cellSize, top, right, bottom, color);
                break;
            case "W":
                canvas.drawRect(left, top, (col + dWidth) * cellSize, bottom, color);
                break;
        }
    }

    private void drawIdentifiedImage(Canvas canvas) {
        log("drawing identified target ids on map");
        RectF rect = null;
        if (obstacleCoor.size() > 0) {
            for (int i = 0; i < obstacleCoor.size(); i++) {
                int col = obstacleCoor.get(i).getObsXCoor();
                int row = this.convertRow(obstacleCoor.get(i).getObsYCoor());
                int targetID = obstacleCoor.get(i).getTargetID();
                if (targetID != -1) {
                    String direction = obstacleCoor.get(i).getDirection();
                    rect = new RectF(col * cellSize, row * cellSize, (col + 1) * cellSize, (row + 1) * cellSize);
                    canvas.drawRect(rect, cells[col][row].paint);
                    canvas.drawText(targetID + "", col * cellSize + cellSize / 2f, row * cellSize + cellSize / 1.4f, targetTextPaint);
                    // draw direction
                    drawDirection(canvas, col, row, direction, imageIdentifiedFacePaint);
                }
            }
        }
    }

    private void drawRobot(Canvas canvas) {
        log("drawing robot on map");
        RectF rect;
        int col = robot.getX();
        int row = this.convertRow(robot.getY());
        int span = 2;
        rect = new RectF((col-1) * cellSize, (row-1) * cellSize, (col + span) * cellSize, (row + span) * cellSize);
        switch (robot.getDirection()) {
            case "N":
                robotDirectionBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.robot_north);
                break;
            case "E":
                robotDirectionBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.robot_east);
                break;
            case "S":
                robotDirectionBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.robot_south);
                break;
            case "W":
                robotDirectionBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.robot_west);
                break;
            default:
                break;
        }
        canvas.drawBitmap(robotDirectionBitmap, null, rect, null);
    }

    public void setRobotCoor(int x, int y, String d) {
        log("setting robot coordinates: (" + x + y + ")");
        int oldX = robot.getX();
        int oldY = this.convertRow(robot.getY());

        // Ensure the robot's coordinates are within the grid boundaries


        if (oldX != -1 && oldY != -1 && oldY!=0 && oldX!=0) {
            for (int i = oldX - 1; i <= oldX + 1; i++)
                for (int j = oldY - 1; j <= oldY + 1; j++)
                    if (!start) {
                        // TODO: Solve bug where path explored makes the program crash if out of bounds
                        //log("CRASHING: " + i + ", " +  j);
                        try {
                            cells[i][j].setType("unexplored");
                        } catch (ArrayIndexOutOfBoundsException e) {
                            // do nothing
                        }
                    } else {
                        // TODO: Solve bug where path explored makes the program crash if out of bounds
//                        java.lang.ArrayIndexOutOfBoundsException: length=21; index=-1
                        // j becomes -1 and crashes... WHY?!?!?!?
                        //log("CRASHING explored: " + i + ", " +  j);
                        try {
                            cells[i][j].setType("explored");
                        } catch (ArrayIndexOutOfBoundsException e) {
                            // do nothing
                        }
                    }

        }

        robot.setX(x);
        robot.setY(y);
        robot.setDirection(d);
        int col = x;
        int row = this.convertRow(y);

        if (col >= 21) {
            col = 20;
        } else if (col < 0) {
            col = 0;
        }

        if (row >= 21) {
            row = 20;
        } else if (row < 0) {
            row = 0;
        }

        log("COLUMN: " + col + " ROW: " + row);
        for (int i = col - 1; i <= col + 1; i++)
            for (int j = row - 1; j <= row + 1; j++)
                if (isRobotWithinCanvasRegion(i, j)) {
                    try {
                        cells[i][j].setType("robot");
                    } catch (ArrayIndexOutOfBoundsException e) {
                        // do nothing
                    }
                } else {
                    //Handle exception when the robot is placed out of bounds
                    Toast.makeText(getContext(), "Robot position is out of bounds!", Toast.LENGTH_SHORT).show();
//                    return;
                }
        this.invalidate();
    }

    public void animateRobotPath(ArrayList<ArrayList<Integer>> path) {
        // Use a handler to post a delayed runnable which will update the robot's position on the map
        final Handler handler = new Handler();
        final int delay = 650; // milliseconds of delay for each step

        for (int i = 0; i < path.size(); i++) {
            int finalI = i;
            handler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    // Extract the x and y coordinates
                    int x = path.get(finalI).get(0);
                    int y = path.get(finalI).get(1);

                    // Update the robot's position on the map
                    setRobotCoor(x + 1, y + 1, robot.getDirection()); // Adjusting for your grid's indexing
                    // Invalidate the map view to trigger a redraw
                    sendRobotStatus();
                    invalidate();
                }
            }, delay * i); // Each iteration is delayed by its index times the delay, creating a sequence over time
        }
    }


    // checks if the cell is occupied
    private boolean checkGridEmpty(int x, int y) {
        if (isWithinCanvasRegion(x, y)) {
            if (cells[x][y].getType() == "robot" || cells[x][y].getType() == "obstacle")
                return false;
            else return true;
        } else return true;
    }

    // checks if there is sufficient space to place a robot
    private boolean checkSpaceEnough(int x, int y) {
        for (int i = x - 1; i <= x + 1; i++)
            for (int j = y - 1; j <= y + 1; j++) {
                if (i >= 0 && i <= NUM_COLS && j >= 0 && j < NUM_ROWS) {
                    if (cells[i][j].getType() == "obstacle") {
                        Toast.makeText(getContext(), "Cell is already occupied!", Toast.LENGTH_SHORT).show();
                        return false;
                    }
                } else {
                    Toast.makeText(getContext(), "OUT OF BOUNDS", Toast.LENGTH_SHORT).show();
                    return false;
                }
            }
        return true;
    }

    public boolean onDragEvent(DragEvent event) {
        switch (event.getAction()) {
            case DragEvent.ACTION_DROP:
                Log.d(TAG, "drop object here");
                // Determine the coordinates of the drop event
                float x = (event.getX() - focusx) / mScaleFactor + focusx;
                float y = (event.getY() - focusy) / mScaleFactor + focusy;

                // Convert the coordinates into grid cell coordinates
                int cellX = (int) (x / cellSize);  // Calculate cell width
                int cellY = this.convertRow((int) (y / cellSize));  // Calculate cell height

                // Check if the drop is within the bounds of your 20x20 grid
                if (isWithinCanvasRegion(cellX, cellY) && checkGridEmpty(cellX, this.convertRow(cellY))) {
                    // handle drop event (place obstacle in grid cell)
                    String obsID = event.getClipData().getItemAt(0).getText().toString();
                    setObstacleCoor(cellX, cellY, obsID);
                    Toast.makeText(getContext(), "Obstacle is placed at (" + (cellX - 1) + ", " + (cellY - 1) + ")", Toast.LENGTH_SHORT).show();
                    // ADDED: send to RPI obstacle details
//                    RpiController.sendToRpi(RpiController.getObstacleDetails(obstacleCoor.get(obstacleCoor.size() - 1)));
                    HomeFragment.modifyObstacleVisibility(Integer.parseInt(obsID) - 1, false);
                    this.invalidate();
                } else {
                    log("out of boundary");
                }
                break;
            default:
                break;
        }
        return true;
    }

    // Map.java (inside the Map class)
    private void addObstacle(int x, int y, String id, String dir) {
        setObstacleCoor(x, y, id);           // x,y are 1..20 in Map
        int row = convertRow(y);              // convert to row index used by cells[][]
        int idx = cells[x][row].getObsIndex();// index of this obstacle in obstacleCoor
        obstacleCoor.get(idx).setDirection(dir); // "N","E","S","W"
    }


    public void setPresetObstacles(String preset) {
        switch (preset) {
            case "Preset 1":
                addObstacle(3,  5,  "1", "S");
                addObstacle(6,  8,  "2", "E");
                addObstacle(9,  12, "3", "N");
                addObstacle(12, 16, "4", "W");
                addObstacle(15, 7,  "5", "S");
                addObstacle(18, 14, "6", "N");
//                addObstacle(7,  19, "7", "W");
//                addObstacle(14, 10, "8", "E");
//                addObstacle(20, 4,  "9", "W");
                break;

            case "Preset 2":
                addObstacle(4,  6,  "1", "E");
                addObstacle(8,  3,  "2", "S");
                addObstacle(11, 9,  "3", "N");
                addObstacle(16, 5,  "4", "W");
                addObstacle(19, 13, "5", "N");
                addObstacle(5,  18, "6", "S");
                addObstacle(13, 15, "7", "E");
                addObstacle(17, 8,  "8", "W");
                break;

            case "Preset 3":
                addObstacle(3,  17, "1", "S");
                addObstacle(6,  15, "2", "E");
                addObstacle(10, 6,  "3", "W");
                addObstacle(12, 12, "4", "N");
                addObstacle(15, 9,  "5", "S");
                addObstacle(18, 3,  "6", "E");
                addObstacle(7,  13, "7", "W");
                addObstacle(14, 18, "8", "N");
                addObstacle(20, 5,  "9", "E");
                addObstacle(8, 9,  "10", "W");

                break;

            case "Preset 4":
                addObstacle(3,  9,  "1", "E");
                addObstacle(5,  13, "2", "S");
                addObstacle(9,  4,  "3", "N");
                addObstacle(13, 7,  "4", "W");
                addObstacle(16, 16, "5", "N");
                addObstacle(19, 6,  "6", "S");
                addObstacle(11, 19, "7", "E");
                addObstacle(6,  20, "8", "W");
                addObstacle(17, 10, "9", "S");
                addObstacle(20, 20,  "10", "S");
                break;

            case "Preset 5":
                addObstacle(4,  18, "1", "N");
                addObstacle(8,  14, "2", "E");
                addObstacle(12, 9,  "3", "S");
                addObstacle(15, 5,  "4", "W");
                addObstacle(20, 12, "5", "N");
                addObstacle(6,  3,  "6", "E");
                addObstacle(10, 17, "7", "W");
                addObstacle(14, 6,  "8", "S");
                addObstacle(18, 20, "9", "E");
                addObstacle(5, 8,  "10", "N");
                break;

            case "Preset 6":
                addObstacle(5,  5,  "1", "W");
                addObstacle(7,  5,  "2", "S");
                addObstacle(11, 11, "3", "N");
                addObstacle(16, 8,  "4", "W");
                addObstacle(19, 4,  "5", "S");
                addObstacle(6,  16, "6", "N");
                addObstacle(13, 20, "7", "W");
                addObstacle(17, 14, "8", "E");
                addObstacle(20, 9,  "9", "N");
                addObstacle(9, 9,  "10", "N");
                break;

//            case "Preset 7":
////                set 7: [[[5, 1], 'east'], [[11, 6], 'south'], [[11, 8], 'north'], [[11, 15], 'east'], [[2, 13], 'north']]
//                setObstacleCoor(5, 1, "1"); // east
//                obstacleCoor.get(cells[5][convertRow(1)].getObsIndex()).setDirection("E");
//
//                setObstacleCoor(11, 6, "2"); // south
//                obstacleCoor.get(cells[11][convertRow(6)].getObsIndex()).setDirection("S");
//
//                setObstacleCoor(11, 8, "3"); // north
//                obstacleCoor.get(cells[11][convertRow(8)].getObsIndex()).setDirection("N");
//
//                setObstacleCoor(11, 15, "4"); // east
//                obstacleCoor.get(cells[11][convertRow(15)].getObsIndex()).setDirection("E");
//
//                setObstacleCoor(2, 13, "5"); // north
//                obstacleCoor.get(cells[2][convertRow(13)].getObsIndex()).setDirection("S");
//
//                setObstacleCoor(17, 7, "6"); // north
//                obstacleCoor.get(cells[17][convertRow(7)].getObsIndex()).setDirection("N");
//
//                setObstacleCoor(18, 15, "7"); // north
//                obstacleCoor.get(cells[18][convertRow(15)].getObsIndex()).setDirection("N");
//
//                setObstacleCoor(2, 17, "8"); // north
//                obstacleCoor.get(cells[2][convertRow(17)].getObsIndex()).setDirection("E");
//                break;

        }
    }


    @Override
    public boolean onTouchEvent(MotionEvent event) {
        if (mScaleDetector != null) mScaleDetector.onTouchEvent(event);

        // Map touch to grid coords (keep your existing math)
        float viewX = event.getX();
        float viewY = event.getY();
        float x = (viewX - focusx) / mScaleFactor + focusx;
        float y = (viewY) / mScaleFactor;
        int cellX = (int) (x / cellSize);
        int cellY = (int) (y / cellSize);

        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN: {
                if (isWithinCanvasRegion(cellX, cellY)) {
                    if (canSetDirection) {
                        String d;
                        if (cells[cellX][cellY].getType() == "obstacle") {
                            d = getNewDirection(obstacleCoor.get(cells[cellX][cellY].getObsIndex()).getDirection());
                            obstacleCoor.get(cells[cellX][cellY].getObsIndex()).setDirection(d);
                            this.invalidate();
                        } else if (cells[cellX][cellY].getType() == "robot") {
                            d = getNewDirection(robot.getDirection());
                            robot.setDirection(d);
                            sendRobotStatus();
                            this.invalidate();
                        }
                    } else if (canDrawRobot) {
                        if (checkSpaceEnough(cellX, cellY)) {
                            setRobotCoor(cellX, this.convertRow(cellY), DEFAULT_DIRECTION);
                            sendRobotStatus();
                        } else {
                            log("already have an object here");
                        }
                    } else if (cells[cellX][cellY].getType() == "obstacle") {
                        // Prepare for tap-to-edit (short tap) while keeping drag intact
                        if (tapToEditEnabled) {
                            potentialTap = true;
                            downX = viewX; downY = viewY;
                            downObstacleIndex = cells[cellX][cellY].getObsIndex();
                        } else {
                            potentialTap = false;
                            downObstacleIndex = -1;
                        }

                        // ORIGINAL drag start (unchanged)
                        currentSelected = cells[cellX][cellY].getObsIndex();
                        log("current selected: " + currentSelected);
                        int oldX = obstacleCoor.get(currentSelected).getObsXCoor();
                        int oldY = this.convertRow(obstacleCoor.get(currentSelected).getObsYCoor());
                        cells[oldX][oldY].setType("unexplored");
                        cells[oldX][oldY].setObsIndex(-1);
                        this.invalidate();
                    }
                }
                return true;
            }

            case MotionEvent.ACTION_MOVE: {
                if (pan) {
                    focusx = viewX;
                    focusy = viewY;
                    this.invalidate();
                }
                // If finger moved, it's a drag, not a tap
                if (potentialTap && (Math.abs(viewX - downX) > touchSlop || Math.abs(viewY - downY) > touchSlop)) {
                    potentialTap = false;
                }
                if (!(currentSelected == -1) && checkGridEmpty(cellX, cellY)) {
                    log("within boundary, can move");
                    obstacleCoor.get(currentSelected).setObsXCoor(cellX);
                    obstacleCoor.get(currentSelected).setObsYCoor(this.convertRow(cellY));
                    this.invalidate();
                }
                return true;
            }

            case MotionEvent.ACTION_UP: {
                // Decide if we should fire a tap-to-edit after finishing original logic
                boolean fireTap = false;
                int tapIdx = -1;
                if (tapToEditEnabled && potentialTap && downObstacleIndex != -1 &&
                        Math.abs(viewX - downX) <= touchSlop && Math.abs(viewY - downY) <= touchSlop) {
                    fireTap = true;
                    tapIdx = downObstacleIndex;
                }

                log("ACTION_UP: (" + cellX + " , " + cellY + ")");
                int tempcellY = NUM_ROWS - 1 - (int) (y / cellSize); // bottom-origin display
                pan = false;

                if (isWithinCanvasRegion(cellX, cellY) && checkGridEmpty(cellX, cellY)) {
                    if (!(currentSelected == -1)) {
                        cells[cellX][cellY].setObsIndex(currentSelected);
                        cells[cellX][cellY].setType("obstacle");
                        Toast.makeText(getContext(),
                                "Obstacle is placed at (" + (cellX - 1) + ", " + (tempcellY) + ")",
                                Toast.LENGTH_SHORT).show();
                        currentSelected = -1;
                        this.invalidate();
                    }
                } else {
                    log("out of boundary");
                    if (!(currentSelected == -1)) {
                        HomeFragment.modifyObstacleVisibility(obstacleCoor.get(currentSelected).getObsID() - 1, true);
                        obstacleCoor.remove(currentSelected);
                        updateObstacleCoor(currentSelected);
                        currentSelected = -1;
                    }
                }

                // Fire tap-to-edit LAST so state is consistent
                if (fireTap && obstacleTapListener != null &&
                        tapIdx >= 0 && tapIdx < obstacleCoor.size()) {
                    obstacleTapListener.onObstacleTapped(obstacleCoor.get(tapIdx));
                }

                // reset tap flags
                potentialTap = false;
                downObstacleIndex = -1;
                return true;
            }
        }
        return true;
    }



    public boolean isWithinCanvasRegion(int x, int y) {
        // check if (x, y) falls within specific bounds of the canvas
        if (x >= 1 && x <= NUM_COLS && y >= 0 && y <= NUM_ROWS) return true;
        else return false;
    }

    public boolean isRobotWithinCanvasRegion(int x, int y) {
        // check if (x, y) falls within specific bounds of the canvas
        if (x >= 0 && x <= 25 && y >= 0 && y <= 25) return true;
        else return false;
    }

    private void setObstacleCoor(int x, int y, String obsID) {
        log("Setting new obstacle coordinates");
        Obstacle obstacle = new Obstacle(x, y, Integer.parseInt(obsID));
        Map.obstacleCoor.add(obstacle);
        int row = this.convertRow(y);
        cells[x][row].setObsIndex(obstacleCoor.size() - 1);
        cells[x][row].setType("obstacle");
    }


    public boolean placeObstacleFromDialog(int obsId, int userX, int userY, String dirLabel) {
        // Dialog uses 0–19; Map stores 1–20
        int gridX = userX + 1;
        int gridY = userY + 1;

        if (!isWithinCanvasRegion(gridX, gridY) || !checkGridEmpty(gridX, gridY)) {
            return false;
        }

        // Create/place the obstacle in the grid/cells
        setObstacleCoor(gridX, gridY, String.valueOf(obsId));

        // Normalize "Up/Right/Down/Left" -> "N/E/S/W"
        String d = "N";
        if ("Up".equalsIgnoreCase(dirLabel)) d = "N";
        else if ("Right".equalsIgnoreCase(dirLabel)) d = "E";
        else if ("Down".equalsIgnoreCase(dirLabel)) d = "S";
        else if ("Left".equalsIgnoreCase(dirLabel)) d = "W";

        // Write direction back to the obstacle we just placed
        int row = convertRow(gridY);
        int idx = cells[gridX][row].getObsIndex();
        obstacleCoor.get(idx).setDirection(d);

        invalidate();
        return true;
    }

    // Finds the index in obstacleCoor for a given obsId; -1 if not present
    private int findObstacleIndexById(int obsId) {
        for (int i = 0; i < obstacleCoor.size(); i++) {
            if (obstacleCoor.get(i).getObsID() == obsId) return i;
        }
        return -1;
    }

    /**
     * Update an existing obstacle with the given ID.
     * x,y come from the dialog (0..19). Internally we store 1..20.
     */
    public boolean updateObstacleById(int obsId, int userX, int userY, String dirLabel) {
        int idx = findObstacleIndexById(obsId);
        if (idx < 0) return false;

        // Convert dialog coords to internal grid + row index
        int newX  = userX + 1;
        int newY  = userY + 1;
        int newRow = convertRow(newY);

        // Bounds check uses the same helpers as elsewhere
        if (!isWithinCanvasRegion(newX, newRow)) return false;  // y here is a row index

        // If moving to a different cell, ensure destination is free or occupied by itself
        boolean destFree = checkGridEmpty(newX, newRow) || cells[newX][newRow].getObsIndex() == idx;
        if (!destFree) return false;

        // Clear old occupancy
        Obstacle o = obstacleCoor.get(idx);
        int oldX = o.getObsXCoor();
        int oldRow = convertRow(o.getObsYCoor());
        cells[oldX][oldRow].setType("unexplored");
        cells[oldX][oldRow].setObsIndex(-1);

        // Move + direction update
        o.setObsXCoor(newX);
        o.setObsYCoor(newY);

        String d = "N";
        if ("Right".equalsIgnoreCase(dirLabel)) d = "E";
        else if ("Down".equalsIgnoreCase(dirLabel)) d = "S";
        else if ("Left".equalsIgnoreCase(dirLabel)) d = "W";
        o.setDirection(d);

        // Mark new cell
        cells[newX][newRow].setType("obstacle");
        cells[newX][newRow].setObsIndex(idx);

        invalidate();
        return true;
    }



    private String getNewDirection(String currentDir) {
        switch (currentDir) {
            case "N":
                return "E";
            case "E":
                return "S";
            case "S":
                return "W";
            case "W":
                return "N";
        }
        return "N";
    }

    private void log(String message) {
        Log.d(TAG, message);
    }

    private int convertRow(int r) {
        return NUM_ROWS - r;
    }

    public void clearGrid() {
        // clear obstacles
        obstacleCoor.clear();
        currentSelected = -1;

        // reset robot
        robot.setX(-1);
        robot.setY(-1);

        // clear grids
        for (int x = 1; x <= NUM_COLS; x++) {
            for (int y = 0; y < NUM_ROWS; y++) {
                if (!cells[x][y].type.equals("unexplored")) {
                    cells[x][y].setType("unexplored");
                }
                if (cells[x][y].getObsIndex() != -1) {
                    cells[x][y].setObsIndex(-1);
                }
            }
        }

        this.invalidate();
    }

    private void updateObstacleCoor(int start) {
        int x, y, index;
        log("updating obstacle arraylist...");
        for (int i = start; i < obstacleCoor.size(); i++) {
            x = obstacleCoor.get(i).getObsXCoor();
            y = this.convertRow(obstacleCoor.get(i).getObsYCoor());
            index = cells[x][y].getObsIndex();
            cells[x][y].setObsIndex(index - 1);
        }
    }

    public void sendMapToRpi() {
        String task = this.getTaskType();
        RpiController.sendToRpi(RpiController.getMapDetails(task, robot, obstacleCoor));
    }

    public void setObsTargetID(int obsID, int imgID) {
        log("updating identified image id to obstacle");
        for (int i = 0; i < obstacleCoor.size(); i++) {
            if (obstacleCoor.get(i).getObsID() == obsID) {
                obstacleCoor.get(i).setTargetID(imgID);
                int x = obstacleCoor.get(i).getObsXCoor();
                int y = this.convertRow(obstacleCoor.get(i).getObsYCoor());
                cells[x][y].setType("image");
                this.invalidate();
                return;
            }
        }
    }

    public Obstacle getObstacle(int obsID) {
        for (int i = 0; i < obstacleCoor.size(); i++) {
            if (obstacleCoor.get(i).getObsID() == obsID) {
                return obstacleCoor.get(i);
            }
        }
        return null;
    }

//    public boolean robotInMap() {
//        return robot.getX() != -1 && robot.getY() != -1;
//    }

    private void sendRobotStatus() {
        String x = Integer.toString(robot.getX() - 1);
        String y = Integer.toString(robot.getY() - 1);
        String status = "robot at (" + x + " , " + y + ") facing " + robot.getDirection();
        Log.d(TAG, "status: " + status);
        Intent intent = new Intent("getStatus");
        intent.putExtra("robot", status);
        LocalBroadcastManager.getInstance(getContext()).sendBroadcast(intent);
    }

    public void setExploredPath(ArrayList<ArrayList<Integer>> path) {
        final int delayMillis = 3000; // Keep this value in mind, might need to adjust it ~ZJ
        final Handler handler = new Handler();
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                // Do something after 5s = 5000ms
                for (int i = 0; i < path.size(); i++) {
                    int row = convertRow(path.get(i).get(1) + 1);
                    int col = path.get(i).get(0) + 1;
                    try {
                        if (cells[col][row].getType() == "unexplored") {
                            cells[col][row].setType("explored");
                            invalidate();
                        }

                    } catch (ArrayIndexOutOfBoundsException e) {
                        // do nothing
                    }

                    // send status
                    String status = "robot at (" + path.get(i).get(0) + " , " + path.get(i).get(1) + ")";
                    Log.d(TAG, "status: " + status);
                    Intent intent = new Intent("getStatus");
                    intent.putExtra("robot", status);
                    LocalBroadcastManager.getInstance(getContext()).sendBroadcast(intent);
                }
            }
        }, delayMillis);
    }


}
