package com.mcm.nativetest

import android.Manifest
import android.content.Intent
import android.content.pm.ActivityInfo
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.net.Uri
import android.os.Bundle
import android.util.Log
import android.view.SurfaceView
import android.view.View
import android.view.Window
import android.view.WindowManager
import android.widget.ImageView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import aruco.CameraParameters
import cameracalibration.CameraCalibrationActivity
import com.android.volley.Request
import com.android.volley.RequestQueue
import com.android.volley.VolleyError
import com.android.volley.toolbox.StringRequest
import com.android.volley.toolbox.Volley
import kotlinx.android.synthetic.main.activity_main.*
import org.opencv.android.*
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2
import org.opencv.core.CvException
import org.opencv.core.CvType
import org.opencv.core.Mat
import java.io.BufferedReader
import java.io.InputStreamReader
import java.net.ServerSocket
import java.util.*


class MainActivity : AppCompatActivity(), CvCameraViewListener2, SensorEventListener {
    companion object {
        private const val TAG = "MainActivity"

        // Used to load the 'native-lib' library on application startup.
        init {
            System.loadLibrary("native-lib")
        }
    }

    lateinit var thresholdOutput: ImageView
    var cameraParameters: CameraParameters? = null

    // This callback only enables the camera once we're connected1
    private val mLoaderCallback: BaseLoaderCallback = object : BaseLoaderCallback(this) {
        override fun onManagerConnected(status: Int) {
            super.onManagerConnected(status)
            if (status == SUCCESS) {
                Log.i(TAG, "OpenCV loaded successfully")
                colored_image_output!!.enableView()
//                colored_image_output!!.enableFpsMeter()
                //                mOpenCvCameraView.setOnTouchListener(MainActivity.this);
            } else {
                super.onManagerConnected(status)
            }
        }
    }

    // Vision process thread stuff
    var visionProcess: VisionProcessThread? = null
        private set
    private var processThread: Thread? = null
    private lateinit var mRgba: Mat
    private lateinit var mColorOutput: Mat

    var queue: RequestQueue? = null

    var manager: SensorManager? = null
    var accelerometer: Sensor? = null
    var gyro: Sensor? = null

    private lateinit var hsvListener: HSVListener

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        requestWindowFeature(Window.FEATURE_NO_TITLE)
        requestedOrientation = ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        setContentView(R.layout.activity_main)
        colored_image_output.setMaxFrameSize(1280, 960)
        colored_image_output.layoutParams.width = 1280;
        colored_image_output.layoutParams.height = 960;
        colored_image_output.visibility = SurfaceView.VISIBLE
        //        mOpenCvCameraView.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_FRONT);
        colored_image_output.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_BACK)
        colored_image_output.setCvCameraViewListener(this)
        thresholdOutput = findViewById(R.id.threshold_image_output)
        manager = getSystemService(SENSOR_SERVICE) as SensorManager
        accelerometer = manager!!.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyro = manager!!.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
//        textView = findViewById(R.id.textView)
//        fpsTextView = findViewById(R.id.fps)
//        textView.setBackgroundColor(Color.WHITE)
//        fpsTextView.setBackgroundColor(Color.WHITE)
        queue = Volley.newRequestQueue(this)
        hsvListener = HSVListener(this)

        try {
            val ss = ServerSocket(9876)
            serverThread = Thread {
                while (!Thread.currentThread().isInterrupted) {
                    val s = ss.accept()
                    val inputStream = s.getInputStream()
                    val outputStream = s.getOutputStream()

                    val reader = BufferedReader(InputStreamReader(inputStream))

//                    var line = reader.readLine()
//                    while(line.isNotEmpty()) {
//                        println(line)
//                        line = reader.readLine()
//                    }
                    val line = reader.readLine()
                    if (line.contains("/driveForward", true)) {
                        println("DRIVING FORWARD")
                    } else if (line.contains("/stop", true)) {
                        println("STOPPING")
                    } else if (line.contains("/reset", true)) {
                        println("RESETTING")
                        visionProcess?.estimator?.reset()
                    } else if (line.contains("/music", true)) {
                        println("MUSIC")
                        val browserIntent = Intent(Intent.ACTION_VIEW, Uri.parse("https://www.youtube.com/watch?v=dQw4w9WgXcQ"))
                        startActivity(browserIntent)
                    }

                    try {
                        val httpResponse = "HTTP/1.1 200 OK\r\n\r\n${Date()}"
                        outputStream.write(httpResponse.toByteArray(charset("UTF-8")))
                    } catch (e: Exception) {
                        e.printStackTrace()
                    }

                    outputStream.close()
                }
            }
            serverThread.start()
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    lateinit var serverThread: Thread

    public override fun onPause() {
        super.onPause()
        if (colored_image_output != null) colored_image_output.disableView()
        if (processThread != null) {
            visionProcess!!.exit = true
            processThread!!.interrupt()
        }
        processThread = null
        manager!!.unregisterListener(this)
    }

    override fun onResume() {
        super.onResume()
        if (ContextCompat.checkSelfPermission(
                this,
                Manifest.permission.CAMERA
            )
            != PackageManager.PERMISSION_GRANTED
        ) {

            // Give first an explanation, if needed.
            if (ActivityCompat.shouldShowRequestPermissionRationale(
                    this,
                    Manifest.permission.CAMERA
                )
            ) {

                // Show an explanation to the user *asynchronously* -- don't block
                // this thread waiting for the user's response! After the user
                // sees the explanation, try again to request the permission.
                Toast.makeText(this, "You should gib perm kthx", Toast.LENGTH_SHORT)
            } else {

                // No explanation needed, we can request the permission.
                ActivityCompat.requestPermissions(
                    this, arrayOf(Manifest.permission.CAMERA),
                    1
                )
            }
        }
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization")
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback)
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!")
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS)
            cameraParameters = CameraParameters()
            cameraParameters!!.readFromFile(applicationContext.filesDir.toString() + CameraCalibrationActivity.DATA_FILEPATH)
            if (!cameraParameters!!.isValid) {
                calibrate(findViewById(R.id.content))
            }
            if (visionProcess == null) visionProcess = VisionProcessThread(cameraParameters)
            visionProcess!!.exit = false
            if (processThread != null) {
                processThread!!.interrupt()
                processThread = null
            }
            processThread = Thread(visionProcess)
            processThread!!.start()
            hsvListener.init()
        }
        manager!!.registerListener(this, gyro, SensorManager.SENSOR_DELAY_GAME)
        manager!!.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_GAME)
    }

    override fun onCameraViewStarted(width: Int, height: Int) {
        mRgba = Mat(height, width, CvType.CV_8UC4)
        mColorOutput = Mat(height, width, CvType.CV_8UC3)
    }

    override fun onCameraViewStopped() {
        mRgba!!.release()
    }

    var lastTime = System.currentTimeMillis()
    var fps = "FPS: "
    override fun onCameraFrame(inputFrame: CvCameraViewFrame): Mat {
        mRgba = inputFrame.rgba()
        visionProcess!!.setInput(mRgba)
        visionProcess!!.getThresholdOutput(mRgba)
        visionProcess!!.getColorOutput(mColorOutput)

//        Core.rotate(mRgba, mRgba, Core.ROTATE_180);
        val now = System.currentTimeMillis()
        val dt = now - lastTime
        lastTime = now
        fps = "FPS: " + 1.0 / (dt / 1000.0)
        try {
            val bmp = Bitmap.createBitmap(mColorOutput.cols(), mColorOutput.rows(), Bitmap.Config.ARGB_8888)
            Utils.matToBitmap(mColorOutput, bmp)
            runOnUiThread { thresholdOutput!!.setImageBitmap(bmp) }
        } catch (e: CvException) {
            Log.d("Exception", e.message!!)
        }
        return mRgba
    }

    fun calibrate(view: View) {
        val i = Intent(applicationContext, CameraCalibrationActivity::class.java)
        startActivity(i)
    }

    fun connectPi(view: View) {
        // Instantiate the RequestQueue.
        val url = "http://192.168.42.14:5800/api/hello"
        val now = System.currentTimeMillis()

        // Request a string response from the provided URL.
        val stringRequest = StringRequest(
            Request.Method.GET, url,
            { response: String? ->
                // Display the first 500 characters of the response string.
//                    textView.setText("Response is: "+ response.substring(0,500));
                textView.text = "DT: " + (System.currentTimeMillis() - now)
            }) { error: VolleyError -> textView!!.text = error.message }

        // Add the request to the RequestQueue.
        queue!!.add(stringRequest)
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event == null) return
        // I have no idea of a better way to do this
        // since the order in which we get readings doesn't seem to be deterministic
        if (event.sensor.type == Sensor.TYPE_GYROSCOPE) {
            val x = event.values[0].toDouble()
//            Log.d("Sensor", "Omega x $x")
            visionProcess!!.omega = x
        } else if (event.sensor.type == Sensor.TYPE_ACCELEROMETER) {
            // X is forward, y is left
            val x = event.values[2].toDouble()
            val y = event.values[1].toDouble()
//            Log.d("Sensor", "Ax $x Ay $y")
        }
        textView!!.text =
//            "Angle: " + (visionProcess!!.estimator.heading.degrees).toInt()
            visionProcess?.estimator?.pose?.toString()

        //        fpsTextView.setText(fps);
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {
        // Ignored
    }
}