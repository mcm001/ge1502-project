package com.mcm.nativetest.poseestimation;

import android.util.Log;
import aruco.Marker;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.estimator.AngleStatistics;
import edu.wpi.first.wpilibj.estimator.UnscentedKalmanFilter;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import org.photonvision.PhotonUtils;
import org.photonvision.vision.pipe.CVPipe;
import org.photonvision.vision.pipe.impl.CalculatePosePipe;

import java.util.List;

public class PoseEstimator {

    long lastUpdateTime = -1;

    private Pose2d cameraPose = new Pose2d();

    // For heading we'll do a kalman filter
    // state = [position], xdot = [omega]
    // xdot = 0x + Iu
//    public KalmanFilter<N1, N1, N1> headingFilter = new KalmanFilter<>(Nat.N1(),
//            Nat.N1(), new LinearSystem<>(VecBuilder.fill(0), VecBuilder.fill(1), VecBuilder.fill(1), VecBuilder.fill(0)),
//            VecBuilder.fill(0.1), VecBuilder.fill(0.5), 0.05);
    UnscentedKalmanFilter<N1, N1, N1> headingFilter =
            new UnscentedKalmanFilter<>(
                    Nat.N1(),
                    Nat.N1(),
                    (x, u) -> u,
                    (x, u) -> x,
                    VecBuilder.fill(0.05),
                    VecBuilder.fill(0.2),
                    AngleStatistics.angleMean(0),
                    AngleStatistics.angleMean(0),
                    AngleStatistics.angleResidual(0),
                    AngleStatistics.angleResidual(0),
                    AngleStatistics.angleAdd(0),
                    0.05);

    // It really do be like that sometimes
    LinearFilter xFilter = LinearFilter.movingAverage(10);
    LinearFilter yFilter = LinearFilter.movingAverage(10);

    public void predict(long time, double omega) {
        long dt = time - lastUpdateTime;
        if (lastUpdateTime < 0) {
            lastUpdateTime = time;
            dt = 50L;
        }
        double doubleDt = dt / 1000.0;
        lastUpdateTime = time;

//        heading = heading.plus(new Rotation2d(omega * dt));
        headingFilter.predict(VecBuilder.fill(omega), doubleDt);
    }

    CalculatePosePipe pipe = new CalculatePosePipe();

    private void correct(Pose2d cameraInField) {
        // Use kalman filter for heading
        headingFilter.correct(VecBuilder.fill(0), VecBuilder.fill(cameraInField.getRotation().getRadians()));

        // Add x/y measurements to our x/y filters
        double newX = xFilter.calculate(cameraInField.getX());
        double newY = yFilter.calculate(cameraInField.getY());

        cameraPose = new Pose2d(newX, newY, new Rotation2d(headingFilter.getXhat(0)));

        System.out.println(cameraInField);
//        Log.i("HeadingEst", String.format("Heading Estimate: %s", headingFilter.getXhat(0) * 180.0 / Math.PI));
    }

    public void correct(List<Marker> markers) {
        pipe.setParams(new CalculatePosePipe.CalculatePosePipeParams(Rotation2d.fromDegrees(0)));
        CVPipe.CVPipeResult<List<Marker>> ret = pipe.run(markers);

//        markerPose = ret.output.get(0).getCameraToTarget();

        for (Marker t : ret.output) {
            Pose2d fieldToGoal = TargetConstants.getMarkerPose(t.getMarkerId());
            if (fieldToGoal == null) continue;

//            Transform2d cameraToTarget = PhotonUtils.estimateCameraToTarget(
//                    t.getCameraToTarget().getTranslation(), fieldToGoal, new Rotation2d(headingFilter.getXhat(0)));
//
//            Pose2d fieldToCamera = PhotonUtils.estimateFieldToCamera(cameraToTarget, fieldToGoal);

            Pose2d fieldToCamera = PhotonUtils.estimateFieldToCamera(t.getCameraToTarget(), fieldToGoal);

            correct(fieldToCamera);
        }
    }

    public void reset() {
        headingFilter.reset();
        Log.i("PoseEstimator", "Pose reset!");
    }

    public Pose2d getPose() {
        return new Pose2d(cameraPose.getTranslation(), new Rotation2d(headingFilter.getXhat(0)));
    }
}