package com.mcm.nativetest;

import android.util.Log;
import aruco.CameraParameters;
import aruco.Marker;
import aruco.MarkerDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;

import java.util.Vector;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static com.mcm.nativetest.MainActivity.TAG;

public class VisionProcessThread implements Runnable {
    private final Lock inmatlock = new ReentrantLock(true);
    public Mat input;
    public Mat output;
    public boolean exit = false;
    private final CameraParameters cameraParameters;
    private final Vector<Marker> detectedMarkers;
    private final Lock markerLock = new ReentrantLock(false);
    private final Lock outmatlock = new ReentrantLock(true);
    private final MarkerDetector mDetector;
    private byte[] lastSerialData;
    public VisionProcessThread(CameraParameters cp) {
        cameraParameters = cp;
        input = new Mat();
        output = new Mat();
        mDetector = new MarkerDetector();
        detectedMarkers = new Vector<>();


//        mDetector.setThresholdParams(9,9);
//        mDetector.setThresholdMethod(MarkerDetector.thresSuppMethod.CANNY);
    }

    public void setInput(Mat frame) {
        inmatlock.lock();
        frame.copyTo(input);
        inmatlock.unlock();
    }

    public void getOutput(Mat frame) {
        if (output.empty()) {
            inmatlock.lock();
            input.copyTo(frame);
            inmatlock.unlock();
            return;
        }
        outmatlock.lock();
        output.copyTo(frame);
        outmatlock.unlock();
    }

    public String getPositionStr() {
        String output = "";
        markerLock.lock();
        // Format for marker: id rvec1 rvec2 rvec3 tvec1 tvec2 tvec3
        for (Marker m : detectedMarkers) {
            output += "id: " + m.getMarkerId() + "\n\n";
            java.util.List<Double> rvec = new Vector<>();
            java.util.List<Double> tvec = new Vector<>();

            Converters.Mat_to_vector_double(m.getRvec(), rvec);
            Converters.Mat_to_vector_double(m.getTvec(), tvec);


//            for (double r : rvec) {
//                output += r+" ";
//            }


            double x = ((Vector<Double>) tvec).elementAt(0);
            double y = ((Vector<Double>) tvec).elementAt(1);
            double z = ((Vector<Double>) tvec).elementAt(2);

            double vert_rad = Math.atan2(z, y);
            double hori_rad = Math.atan2(z, x);

            output += "Vertical: " + ((360 / 6.28 * vert_rad)) + "\n";
            output += "Horizontal: " + ((360 / 6.28 * hori_rad)) + "\n";

//            for (double r : tvec) {
//                output += r+"\n";
//            }
            output += "\n";
        }
        markerLock.unlock();
        return output;
    }

    public byte[] getSerialBytes() {
        // Output is 2 bytes. First byte is the position of the pan servo, second byte is the position of the tilt servo
        if (!detectedMarkers.isEmpty()) {
            java.util.List<Double> tvec = new Vector<>();
            Converters.Mat_to_vector_double(detectedMarkers.firstElement().getTvec(), tvec);
            double x = ((Vector<Double>) tvec).elementAt(0);
            double y = ((Vector<Double>) tvec).elementAt(1);
            double z = ((Vector<Double>) tvec).elementAt(2);

            double vert_rad = Math.atan2(z, y);
            double hori_rad = Math.atan2(z, x);

            byte x_byte = (byte) (255 - (int) (127 + (hori_rad / 6.28 * 255)));
            byte y_byte = (byte) (int) (vert_rad / 6.28 * 255);
            lastSerialData = new byte[]{x_byte, y_byte};
        }
        return lastSerialData;
    }

    @Override
    public void run() {
        while (!exit) {
            inmatlock.lock();
            Mat mRgba = new Mat();
            input.copyTo(mRgba);
            inmatlock.unlock();
            if (mRgba.empty()) {
                continue;
            }

            if (cameraParameters.isValid()) {
                long dt = System.currentTimeMillis();
                markerLock.lock();
//                mDetector.detect(mRgba, detectedMarkers, cameraParameters, 0.1f);
                mDetector.detect(mRgba, detectedMarkers, cameraParameters, 0.1f);

//                mRgba.convertTo(mRgba, CvType.CV_8UC3);
//                Core.rotate(mRgba, mRgba, Core.ROTATE_90_CLOCKWISE);
//                mRgba.convertTo(mRgba, CvType.CV_8UC4);

                Log.e(TAG, "Detection Time: " + (System.currentTimeMillis() - dt));
                Log.e(TAG, mRgba.width() + " by " + mRgba.height());


                for (Marker m : detectedMarkers) {
                    m.draw3dAxis(mRgba, cameraParameters, new Scalar(0, 255, 0));
                    m.draw3dCube(mRgba, cameraParameters, new Scalar(0, 255, 0));
                    m.draw(mRgba, new Scalar(255, 0, 0), 2, true);
                }
                markerLock.unlock();


                Log.e(TAG, "Marker count: " + detectedMarkers.size());
            } else {
                Imgproc.putText(mRgba, "Invalid Camera Parameters", new Point(mRgba.width() / 4, mRgba.height() / 2), Core.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 255, 0));
            }
//            Imgproc.putText(mRgba, ""+FPS, new Point(10, 10), Core.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0));
            outmatlock.lock();
            mRgba.copyTo(output);
            outmatlock.unlock();

        }


    }
}
