package com.mcm.nativetest

import android.Manifest
import android.content.*
import android.content.pm.ActivityInfo
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.media.MediaPlayer
import android.net.Uri
import android.os.Bundle
import android.os.Handler
import android.os.IBinder
import android.os.Message
import android.provider.Settings
import android.text.method.ScrollingMovementMethod
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
import com.android.volley.RequestQueue
import com.android.volley.toolbox.Volley
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.util.Units
import kotlinx.android.synthetic.main.activity_main.*
import kotlinx.coroutines.*
import org.opencv.android.*
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2
import org.opencv.core.CvException
import org.opencv.core.CvType
import org.opencv.core.Mat
import java.io.BufferedReader
import java.io.InputStreamReader
import java.lang.ref.WeakReference
import java.net.ServerSocket
import java.nio.charset.StandardCharsets
import java.util.*


class MainActivity : AppCompatActivity(), CvCameraViewListener2, SensorEventListener {
    companion object {
        private const val TAG = "MainActivity"

        // Used to load the 'native-lib' library on application startup.
        init {
            System.loadLibrary("native-lib")
        }
    }

    private lateinit var controllerTask: Job

    /*
         * Notifications from UsbService will be received here.
         */
    private val mUsbReceiver: BroadcastReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            when (intent.action) {
                UsbService.ACTION_USB_PERMISSION_GRANTED -> Toast.makeText(context, "USB Ready", Toast.LENGTH_SHORT)
                    .show()
                UsbService.ACTION_USB_PERMISSION_NOT_GRANTED -> Toast.makeText(
                    context,
                    "USB Permission not granted",
                    Toast.LENGTH_SHORT
                ).show()
                UsbService.ACTION_USB_ATTACHED -> {
                    Toast.makeText(context, "USB connected!", Toast.LENGTH_SHORT).show()
//                    status.text = getString(R.string.connected)
                }
                UsbService.ACTION_NO_USB -> Toast.makeText(context, "No USB connected", Toast.LENGTH_SHORT).show()
                UsbService.ACTION_USB_DISCONNECTED -> {
                    Toast.makeText(context, "USB disconnected", Toast.LENGTH_SHORT)
                        .show()
//                    status.text = getString(R.string.disconnected)
                }
                UsbService.ACTION_USB_NOT_SUPPORTED -> Toast.makeText(
                    context,
                    "USB device not supported",
                    Toast.LENGTH_SHORT
                ).show()
            }
        }
    }

    /*
     * This handler will be passed to UsbService. Data received from serial port is displayed through this handler
     */
    private class MyHandler(activity: MainActivity) : Handler() {
        private val mActivity: WeakReference<MainActivity> = WeakReference(activity)
        override fun handleMessage(msg: Message) {
            when (msg.what) {
                UsbService.MESSAGE_FROM_SERIAL_PORT -> {
                    val data = msg.obj as String
                    mActivity.get()!!.addRx("Rx: $data")
                }
                UsbService.CTS_CHANGE -> Toast.makeText(mActivity.get(), "CTS_CHANGE", Toast.LENGTH_LONG).show()
                UsbService.DSR_CHANGE -> Toast.makeText(mActivity.get(), "DSR_CHANGE", Toast.LENGTH_LONG).show()
            }
        }
    }

    private fun addRx(str: String) {
        val newStr = "${serialLog.text}$str"
        var lines = newStr.split("\n")
        if (lines.size > 10) {
            lines = lines.subList(lines.size - 10, lines.size)
        }
        serialLog.text = lines.joinToString("\n")
    }

    private fun sendData(data: String) {
        val bytes = data.toByteArray(charset = StandardCharsets.US_ASCII)
        val success = usbService?.write(bytes)
//        if(success == true) {
//            addRx("TX: $data")
//        }
    }

    var usbService: UsbService? = null
    private var mHandler: MyHandler? = null
    private val usbConnection: ServiceConnection = object : ServiceConnection {
        override fun onServiceConnected(arg0: ComponentName, arg1: IBinder) {
            usbService = (arg1 as UsbService.UsbBinder).service
            usbService?.setHandler(mHandler)
        }

        override fun onServiceDisconnected(arg0: ComponentName) {
            usbService = null
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
    private var mRgba: Mat? = null
    private lateinit var mColorOutput: Mat

    var queue: RequestQueue? = null

    var sensorManager: SensorManager? = null
    var gyro: Sensor? = null

    private lateinit var hsvListener: HSVListener
    private lateinit var controller: Controller

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        requestWindowFeature(Window.FEATURE_NO_TITLE)
        requestedOrientation = ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        mHandler = MyHandler(this) // USB handler
        setContentView(R.layout.activity_main)
        colored_image_output.setMaxFrameSize(1280, 960)
        colored_image_output.layoutParams.width = 1280;
        colored_image_output.layoutParams.height = 960;
        colored_image_output.visibility = SurfaceView.VISIBLE
        colored_image_output.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_FRONT);
//        colored_image_output.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_BACK)
        colored_image_output.setCvCameraViewListener(this)
        thresholdOutput = findViewById(R.id.threshold_image_output)
        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
        gyro = sensorManager!!.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
//        textView = findViewById(R.id.textView)
//        fpsTextView = findViewById(R.id.fps)
//        textView.setBackgroundColor(Color.WHITE)
//        fpsTextView.setBackgroundColor(Color.WHITE)
        queue = Volley.newRequestQueue(this)
        hsvListener = HSVListener(this)
        controller =
            Controller({ sendData("${-it.rightMetersPerSecond}, ${-it.leftMetersPerSecond}\n") }) { visionProcess?.estimator?.pose }


        serialLog.movementMethod = ScrollingMovementMethod()
        serialLog.text = ""


        startServer()
        controllerTask = GlobalScope.launch {
            while (true) {
                controller.update(); yield()
            }
        }
    }

    lateinit var serverThread: Thread
    private lateinit var alarmPlayer: MediaPlayer

    private fun startServer() {
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
                    when {
                        line.contains("/driveForward", true) -> {
                            println("DRIVING FORWARD")
//                            controller.currentState = Controller.State.Power(1.0, 1.0)
//                            GlobalScope.launch {
////                                controller.turnToFace(Pose2d())
////                                while (!controller.currentState.isDone() && controller.currentState != Controller.State.Nothing) {
////                                    delay(10)
////                                }
                                controller.currentState = Controller.State.Power(1.0, 0.8)
//                            controller.currentState = Controller.State.Power(-1.0, 1.0)
////                                delay(1000)
////                                controller.currentState = Controller.State.Nothing
//
////                                while(!controller.currentState.isDone() && controller.currentState != Controller.State.Nothing) {
////                                    delay(10)
////                                }
//
//                                // Wait for a target to show up
//                                // I'm really lazy so all this does is wait to see if
//                                // the pipeline sees a triangle
//                                // If it has been long enough, the robot plays a sound
//                                // and just keeps driving
//                                var targetCount = 0
//                                var i = 0
//
//                                runOnUiThread {
//                                    Toast.makeText(this@MainActivity, "Looking for items", 3)
//                                }
//
//                                while (++i in 0..1000 && targetCount <= 4) {
//                                    delay(20)
//                                    println("hi")
//                                    if (visionProcess?.hasTargets() == true) targetCount++
//                                    else if (targetCount > 0) targetCount--
//                                    println(targetCount)
////                                    if(targetCount > 4) break
//                                }
//                                if (targetCount > 4) {
//                                    // Yell at people. for now, we just print
//                                    println("Found bad thing")
//                                    alarmPlayer =
//                                        MediaPlayer.create(this@MainActivity, Settings.System.DEFAULT_ALARM_ALERT_URI)
//                                    runOnUiThread {
//                                        Toast.makeText(this@MainActivity, "Illegal item!", 5)
//                                    }
//                                    alarmPlayer.start()
//                                    delay(3000)
//                                    alarmPlayer.stop()
//                                } else {
//                                    // Don't yell at them
//                                    println("Understandable")
//                                    alarmPlayer =
//                                        MediaPlayer.create(this@MainActivity, Settings.System.DEFAULT_NOTIFICATION_URI)
//                                    runOnUiThread {
//                                        Toast.makeText(this@MainActivity, "Carry on!", 5)
//                                    }
//                                    alarmPlayer.start()
//                                    delay(3000)
//                                    alarmPlayer.stop()
//                                }
//                            }
                        }
                        line.contains("/stop", true) -> {
                            println("STOPPING")
                            controller.currentState = Controller.State.Nothing
                        }
                        line.contains("/reset", true) -> {
                            println("RESETTING")
                            visionProcess?.estimator?.reset()
                        }
                        line.contains("/music", true) -> {
                            println("MUSIC")
                            val browserIntent =
                                Intent(Intent.ACTION_VIEW, Uri.parse("https://www.youtube.com/watch?v=dQw4w9WgXcQ"))
                            startActivity(browserIntent)
                        }
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

    public override fun onPause() {
        super.onPause()
        if (colored_image_output != null) colored_image_output.disableView()
        if (processThread != null) {
            processThread!!.interrupt()
        }
        processThread = null

        sensorManager!!.unregisterListener(this)
        unregisterReceiver(mUsbReceiver)
        unbindService(usbConnection)
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

            // needs to come after we load camera parameters
            if (visionProcess == null) visionProcess = VisionProcessThread(cameraParameters)

            if (processThread != null) {
                processThread!!.interrupt()
                processThread = null
            }
            processThread = Thread { while (!Thread.currentThread().isInterrupted) visionProcess?.run() }
//            processThread!!.start()
            hsvListener.init()
        }
        sensorManager!!.registerListener(this, gyro, SensorManager.SENSOR_DELAY_GAME)


        setFilters() // Start listening notifications from UsbService
        startService(
            UsbService::class.java,
            usbConnection,
            null
        ) // Start UsbService(if it was not started before) and Bind it
    }

    private fun startService(service: Class<*>, serviceConnection: ServiceConnection, extras: Bundle?) {
        if (!UsbService.SERVICE_CONNECTED) {
            val startService = Intent(this, service)
            if (extras != null && !extras.isEmpty) {
                val keys = extras.keySet()
                for (key in keys) {
                    val extra = extras.getString(key)
                    startService.putExtra(key, extra)
                }
            }
            startService(startService)
        }
        val bindingIntent = Intent(this, service)
        bindService(bindingIntent, serviceConnection, BIND_AUTO_CREATE)
    }

    private fun setFilters() {
        val filter = IntentFilter()
        filter.addAction(UsbService.ACTION_USB_PERMISSION_GRANTED)
        filter.addAction(UsbService.ACTION_NO_USB)
        filter.addAction(UsbService.ACTION_USB_DISCONNECTED)
        filter.addAction(UsbService.ACTION_USB_ATTACHED)
        filter.addAction(UsbService.ACTION_USB_NOT_SUPPORTED)
        filter.addAction(UsbService.ACTION_USB_PERMISSION_NOT_GRANTED)
        registerReceiver(mUsbReceiver, filter)
    }

    override fun onCameraViewStarted(width: Int, height: Int) {
        mColorOutput = Mat(height, width, CvType.CV_8UC3)
    }

    override fun onCameraViewStopped() {
        mRgba?.release()
    }

    var lastTime = System.currentTimeMillis()
    var fps = "FPS: "
    override fun onCameraFrame(inputFrame: CvCameraViewFrame): Mat? {
        mRgba?.release()
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
            runOnUiThread { thresholdOutput.setImageBitmap(bmp) }
        } catch (e: CvException) {
            Log.d("Exception", e.message!!)
        }
        return mRgba
    }

    fun calibrate(view: View) {
        val i = Intent(applicationContext, CameraCalibrationActivity::class.java)
        startActivity(i)
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
            visionProcess?.estimator?.pose
                ?.let { Pose2d(it.translation.times(Units.metersToInches(1.0)), it.rotation) }
                .toString()

        //        fpsTextView.setText(fps);
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {
        // Ignored
    }

    fun tryConnectUsb(view: View) {
        usbService?.findSerialPortDevice()
    }
}