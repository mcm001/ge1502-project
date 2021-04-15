package com.mcm.nativetest;

import aruco.CameraParameters;
import aruco.Marker;
import aruco.MarkerDetector;
import com.mcm.nativetest.poseestimation.PoseEstimator;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import org.apache.commons.lang3.tuple.Pair;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.photonvision.vision.frame.Frame;
import org.photonvision.vision.frame.FrameDivisor;
import org.photonvision.vision.frame.FrameStaticProperties;
import org.photonvision.vision.opencv.CVMat;
import org.photonvision.vision.pipe.impl.Draw2dCrosshairPipe;
import org.photonvision.vision.pipe.impl.Draw2dTargetsPipe;
import org.photonvision.vision.pipe.impl.OutputMatPipe;
import org.photonvision.vision.pipe.impl.ResizeImagePipe;
import org.photonvision.vision.pipeline.ReflectivePipeline;
import org.photonvision.vision.pipeline.ReflectivePipelineSettings;
import org.photonvision.vision.pipeline.result.CVPipelineResult;
import org.photonvision.vision.target.TrackedTarget;

import java.util.Collections;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class VisionProcessThread implements Runnable {
    private final Lock inmatlock = new ReentrantLock(true);
    private Mat input;
    private Mat output;
    private Mat colorOutput = new Mat();
    private long inTime = 0;
    private final CameraParameters cameraParameters;
    public final Vector<Marker> detectedMarkers;
    private final Lock markerLock = new ReentrantLock(false);
    private final Lock outmatlock = new ReentrantLock(true);
    private final MarkerDetector mDetector;

    public final PoseEstimator estimator = new PoseEstimator();
    public double ax, ay, omega;
    private List<TrackedTarget> result = Collections.emptyList();
    private final Object resultLock = new Object();
    private AtomicBoolean hasTargets = new AtomicBoolean();

    public VisionProcessThread(CameraParameters cp) {
        cameraParameters = cp;
        input = new Mat();
        output = new Mat();
        mDetector = new MarkerDetector();
        detectedMarkers = new Vector<>();

//        mDetector.setThresholdParams(9,9);
//        mDetector.setThresholdMethod(MarkerDetector.thresSuppMethod.CANNY);

        // Some sane defaults
        setParams();
    }

    public void setInput(Mat frame) {
        inmatlock.lock();
        input.release();
        frame.copyTo(input);
        inTime = System.currentTimeMillis();
        inmatlock.unlock();
    }

    public void getThresholdOutput(Mat frame) {
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

    public void getColorOutput(Mat frame) {
        if (colorOutput.empty()) {
            inmatlock.lock();
            input.copyTo(frame);
            inmatlock.unlock();
            return;
        }
        outmatlock.lock();
        colorOutput.copyTo(frame);
        outmatlock.unlock();
    }

    Mat mRgba = new Mat();

    @Override
    public void run() {
        inmatlock.lock();
        input.copyTo(mRgba);
        inmatlock.unlock();
        if (mRgba.empty()) {
            return;
        }

        // Predict at the start
        long now = System.currentTimeMillis();
        estimator.predict(now, omega);

        // These internally mutate mRgba
        detectMarkers();
        detectShapes();
        drawMarkers();

        outmatlock.lock();
        mRgba.copyTo(output);
        mRgba.release();
        outmatlock.unlock();

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    ReflectivePipeline visionPipeline = new ReflectivePipeline();
    FrameStaticProperties props;

    private final Draw2dTargetsPipe draw2dTargetsPipe = new Draw2dTargetsPipe();
    private final ResizeImagePipe resizeImagePipe = new ResizeImagePipe();
    private final Draw2dCrosshairPipe draw2dCrosshairPipe = new Draw2dCrosshairPipe();
    private final OutputMatPipe outputMatPipe = new OutputMatPipe();

    private void setParams() {
        ReflectivePipelineSettings settings = visionPipeline.getSettings();
        settings.outputShowMultipleTargets = true;
        settings.streamingFrameDivisor = FrameDivisor.NONE;

        props = new FrameStaticProperties(1280, 960, 90, Rotation2d.fromDegrees(0), null);
        resizeImagePipe.setParams(new ResizeImagePipe.ResizeImageParams(settings.streamingFrameDivisor));
        draw2dTargetsPipe.setParams(new Draw2dTargetsPipe.Draw2dTargetsParams(true, true, settings.streamingFrameDivisor));
        draw2dCrosshairPipe.setParams(new Draw2dCrosshairPipe.Draw2dCrosshairParams(props, settings.streamingFrameDivisor));
        outputMatPipe.setParams(new OutputMatPipe.OutputMatParams());
    }

    private void detectShapes() {
        inmatlock.lock();
        outmatlock.lock();

        colorOutput.release();

        Imgproc.cvtColor(mRgba, mRgba, Imgproc.COLOR_RGBA2BGR);
//        Imgproc.line(mRgba, new Point(0, 0), new Point(1000, 1000), new Scalar(1, 1, 1), 10);
//        Imgproc.circle(mRgba, new Point(300, 300), 100, new Scalar(0, 0, 150), Core.FILLED);
//        if (true) return;

        Frame f = new Frame(new CVMat(mRgba), props);
        CVPipelineResult result = visionPipeline.run(f);

        Mat inMat = result.inputFrame.image.getMat();
        Mat outMat = result.outputFrame.image.getMat();

        List<TrackedTarget> targetsToDraw = result.targets;

        resizeImagePipe.run(inMat);
        draw2dCrosshairPipe.run(Pair.of(inMat, targetsToDraw));
        draw2dTargetsPipe.run(Pair.of(inMat, targetsToDraw));

        resizeImagePipe.run(outMat);
        outputMatPipe.run(outMat);
        draw2dCrosshairPipe.run(Pair.of(outMat, targetsToDraw));
        draw2dTargetsPipe.run(Pair.of(outMat, targetsToDraw));

        inMat.copyTo(colorOutput);
        outMat.copyTo(mRgba);

        Imgproc.cvtColor(mRgba, mRgba, Imgproc.COLOR_BGR2RGBA);
        Imgproc.cvtColor(colorOutput, colorOutput, Imgproc.COLOR_BGR2RGB);

        synchronized (resultLock) {
            if (this.result != null)
                this.result.forEach(TrackedTarget::release);
            this.result = result.targets;
            hasTargets.set(result.hasTargets());
        }

        inmatlock.unlock();
        outmatlock.unlock();
    }

    private void detectMarkers() {
        if (cameraParameters.isValid()) {
            markerLock.lock();
            mDetector.detect(mRgba, detectedMarkers, cameraParameters, 0.1f);
            markerLock.unlock();

            processMarkers(detectedMarkers);

        } else {
            Imgproc.putText(mRgba, "Invalid Camera Parameters", new Point(mRgba.width() / 4, mRgba.height() / 2), Core.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 255, 0));
        }
    }

    private void drawMarkers() {
        if (cameraParameters.isValid()) {
            markerLock.lock();
            for (Marker m : detectedMarkers) {
                m.draw3dAxis(mRgba, cameraParameters, new Scalar(0, 255, 0));
                m.draw3dCube(mRgba, cameraParameters, new Scalar(0, 255, 0));
                m.draw(mRgba, new Scalar(255, 0, 0), 2, true);

                m.draw3dAxis(colorOutput, cameraParameters, new Scalar(0, 255, 0));
                m.draw3dCube(colorOutput, cameraParameters, new Scalar(0, 255, 0));
                m.draw(colorOutput, new Scalar(255, 0, 0), 2, true);
            }
            detectedMarkers.forEach(Marker::release);
            markerLock.unlock();
        } else {
            Imgproc.putText(mRgba, "Invalid Camera Parameters", new Point(mRgba.width() / 4, mRgba.height() / 2), Core.FONT_HERSHEY_SIMPLEX, 2, new Scalar(0, 255, 0));
        }
    }

    private void processMarkers(List<Marker> markers) {

        if (!markers.isEmpty()) {
            estimator.correct(markers);
        }
    }

    public ReflectivePipelineSettings getPipelineSettings() {
        return visionPipeline.getSettings();
    }

    public void setSettings(ReflectivePipelineSettings settings) {
        visionPipeline.setSettings(settings);
    }

    public boolean hasTargets() {
        return hasTargets.get();
    }
}
