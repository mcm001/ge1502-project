package com.mcm.nativetest;

import android.Manifest;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceView;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Toast;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import aruco.CameraParameters;
import cameracalibration.CameraCalibrationActivity;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.io.BufferedOutputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;

public class MainActivity extends AppCompatActivity implements CameraBridgeViewBase.CvCameraViewListener2, SensorEventListener {

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    protected static final String TAG = "MainActivity";

    CameraBridgeViewBase mOpenCvCameraView;
    CameraParameters cameraParameters;

    // This callback only enables the camera once we're connected1
    private final BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            if (status == LoaderCallbackInterface.SUCCESS) {
                Log.i(TAG, "OpenCV loaded successfully");
                mOpenCvCameraView.enableView();
//                mOpenCvCameraView.enableFpsMeter();
//                mOpenCvCameraView.setOnTouchListener(MainActivity.this);
            } else {
                super.onManagerConnected(status);
            }
        }
    };

    // Vision process thread stuff
    private VisionProcessThread visionProcess;
    private Thread processThread;
    private Mat mRgba;

    SensorManager manager;
    Sensor accelerometer, gyro;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);

        mOpenCvCameraView = findViewById(R.id.color_blob_detection_activity_surface_view);
        mOpenCvCameraView.setMaxFrameSize(1280, 960);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);

        manager = (SensorManager) getSystemService(SENSOR_SERVICE);
        accelerometer = manager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyro = manager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
    }

    @Override
    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        if(processThread != null) {
            visionProcess.exit = true;
            processThread.interrupt();
        }
        processThread = null;

        manager.unregisterListener(this);
    }

    @Override
    protected void onResume() {
        super.onResume();

        if (ContextCompat.checkSelfPermission(this,
                Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {

            // Give first an explanation, if needed.
            if (ActivityCompat.shouldShowRequestPermissionRationale(this,
                    Manifest.permission.CAMERA)) {

                // Show an explanation to the user *asynchronously* -- don't block
                // this thread waiting for the user's response! After the user
                // sees the explanation, try again to request the permission.
                Toast.makeText(this,  "You should gib perm kthx", Toast.LENGTH_SHORT);
            } else {

                // No explanation needed, we can request the permission.
                ActivityCompat.requestPermissions(this,
                        new String[]{Manifest.permission.CAMERA},
                        1);
            }
        }

        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
            cameraParameters = new CameraParameters();
            cameraParameters.readFromFile(getApplicationContext().getFilesDir() + CameraCalibrationActivity.DATA_FILEPATH);
            if (!cameraParameters.isValid()) {
                calibrate(findViewById(R.id.content));
            }
            if(visionProcess == null) visionProcess = new VisionProcessThread(cameraParameters);
            visionProcess.exit = false;
            if (processThread != null) {
                processThread.interrupt();
                processThread = null;
            }
            processThread = new Thread(visionProcess);
            processThread.start();
        }

        manager.registerListener(this, gyro, SensorManager.SENSOR_DELAY_GAME);
        manager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_GAME);
    }

    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
    }

    public void onCameraViewStopped() {
        mRgba.release();
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();
        visionProcess.setInput(mRgba);
        visionProcess.getOutput(mRgba);

        return mRgba;
    }

    public void calibrate(View view) {
        Intent i = new Intent(getApplicationContext(), CameraCalibrationActivity.class);
        startActivity(i);
    }

    public void connectPi(View view) {
        try {
            URL host = new URL("http", "raspberrypi.local", "5555");
            HttpURLConnection c = (HttpURLConnection) host.openConnection();

            c.setDoOutput(true);
            c.setRequestMethod("PUT");

            OutputStream out = new BufferedOutputStream(c.getOutputStream());
            out.write("Hello".getBytes(StandardCharsets.UTF_8));
            out.close(); // Close will flush
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if(event == null) return;
        // I have no idea of a better way to do this
        // since the order in which we get readings doesn't seem to be deterministic
        if(event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            double x = event.values[0];
            Log.i("Sensor", "Omega x " + x);
        }
        else if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            // X is forward, y is left
            double x = event.values[2];
            double y = event.values[1];
            Log.i("Sensor", "Ax " + x + " Ay " + y);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Ignored
    }
}
