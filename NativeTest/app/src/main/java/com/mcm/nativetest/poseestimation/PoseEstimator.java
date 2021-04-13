package com.mcm.nativetest.poseestimation;

import aruco.Marker;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import org.photonvision.PhotonUtils;
import org.photonvision.vision.pipe.CVPipe;
import org.photonvision.vision.pipe.impl.CalculatePosePipe;

import java.util.List;

public class PoseEstimator {
    // Let's just do a median filter on x/y position, because the accelerometer readings kinda suck

    // For heading we'll do a kalman filter
    // state = [position], xdot = [omega]
    // xdot = 0x + 1u
    public KalmanFilter<N1, N1, N1> headingFilter = new KalmanFilter<>(Nat.N1(),
            Nat.N1(), new LinearSystem<>(VecBuilder.fill(0), VecBuilder.fill(1), VecBuilder.fill(1), VecBuilder.fill(0)),
            VecBuilder.fill(0.1), VecBuilder.fill(0.1), 0.05);

    // It really do be like that sometimes
    LinearFilter xFilter = LinearFilter.movingAverage(10);
    LinearFilter yFilter = LinearFilter.movingAverage(10);

    public Pose2d poseEstimate = new Pose2d();

    double lastUpdateTime = -1;

    public void predict(double time, double ax, double ay, double omega) {
        double now = System.currentTimeMillis();
        double dt = now - lastUpdateTime;
        if (lastUpdateTime < 0) {
            lastUpdateTime = System.currentTimeMillis();
            dt = 50.0;
        }
        dt /= 1000.0;
        lastUpdateTime = now;

        headingFilter.predict(VecBuilder.fill(omega), dt);
//        Log.i("HeadingEst", String.format("Heading Estimate: %s", headingFilter.getXhat(0)));
    }

    private void correct(Pose2d cameraInField) {
        // rotate our camera pose until the error is less than +-180
        double heading = cameraInField.getRotation().getRadians();
        double xhat = headingFilter.getXhat(0);
        while(heading - xhat > 180) heading -= 360;
        while(heading - xhat < -180) heading += 360;

        // Use kalman filter for heading
        headingFilter.correct(VecBuilder.fill(0), VecBuilder.fill(cameraInField.getRotation().getRadians()));

        // Add x/y measurements to our x/y filters
        double newX = xFilter.calculate(cameraInField.getX());
        double newY = xFilter.calculate(cameraInField.getY());

        poseEstimate = new Pose2d(newX, newY, new Rotation2d(headingFilter.getXhat(0)));

        System.out.println(cameraInField);
//        Log.i("HeadingEst", String.format("Heading Estimate: %s", headingFilter.getXhat(0) * 180.0 / Math.PI));

    }

    CalculatePosePipe pipe = new CalculatePosePipe();

    public void correct(List<Marker> targetList) {
        pipe.setParams(new CalculatePosePipe.CalculatePosePipeParams(Rotation2d.fromDegrees(0)));
        CVPipe.CVPipeResult<List<Marker>> ret = pipe.run(targetList);

        for (Marker t : ret.output) {
            Pose2d fieldToGoal = TargetConstants.getMarkerPose(t.getMarkerId());
            if (fieldToGoal == null) continue;

            Transform2d cameraToTarget = PhotonUtils.estimateCameraToTarget(
                    t.getCameraToTarget().getTranslation(), fieldToGoal, new Rotation2d(headingFilter.getXhat(0)));

            Pose2d fieldToCamera = PhotonUtils.estimateFieldToCamera(cameraToTarget, fieldToGoal);
            correct(fieldToCamera);
        }
    }
}
