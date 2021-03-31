package com.mcm.nativetest;

import aruco.CameraParameters;
import aruco.Marker;
import aruco.MarkerDetector;
import com.mcm.nativetest.poseestimation.PoseEstimator;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Vector;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class VisionProcessThread implements Runnable {
    private final Lock inmatlock = new ReentrantLock(true);
    private Mat input;
    private Mat output;
    private long inTime = 0;
    public boolean exit = false;
    private final CameraParameters cameraParameters;
    private final Vector<Marker> detectedMarkers;
    private final Lock markerLock = new ReentrantLock(false);
    private final Lock outmatlock = new ReentrantLock(true);
    private final MarkerDetector mDetector;

    private final PoseEstimator estimator = new PoseEstimator();
    private double ax, ay, omega;

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
        input.release();
        frame.copyTo(input);
        inTime = System.currentTimeMillis();
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

    Mat mRgba = new Mat();

    @Override
    public void run() {
        while (!exit) {
            inmatlock.lock();
            input.copyTo(mRgba);
            inmatlock.unlock();
            if (mRgba.empty()) {
                continue;
            }

            if (cameraParameters.isValid()) {
                markerLock.lock();
                mDetector.detect(mRgba, detectedMarkers, cameraParameters, 0.1f);

                for (Marker m : detectedMarkers) {
                    m.draw3dAxis(mRgba, cameraParameters, new Scalar(0, 255, 0));
                    m.draw3dCube(mRgba, cameraParameters, new Scalar(0, 255, 0));
                    m.draw(mRgba, new Scalar(255, 0, 0), 2, true);
                }
                markerLock.unlock();

                processMarkers(detectedMarkers, inTime / 1000.0);

                detectedMarkers.forEach(Marker::release);
            } else {
                Imgproc.putText(mRgba, "Invalid Camera Parameters", new Point(mRgba.width() / 4, mRgba.height() / 2), Core.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 255, 0));
            }
            outmatlock.lock();
            mRgba.copyTo(output);
            mRgba.release();
            outmatlock.unlock();
        }
    }

    private void processMarkers(List<Marker> m, double detectionTimeSec) {
        double now = System.currentTimeMillis() / 1000.0;
        estimator.update(now, ax, ay, omega);

        if(!m.isEmpty())
            estimator.correct(m, detectionTimeSec);
    }
}
