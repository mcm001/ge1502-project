package com.mcm.nativetest.poseestimation;

import android.util.Log;
import aruco.Marker;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import org.photonvision.PhotonUtils;
import org.photonvision.vision.pipe.CVPipe;
import org.photonvision.vision.pipe.impl.CalculatePosePipe;

import java.util.List;

public class PoseEstimator {

//    /** States: [x, y, vx, vy, heading]^T
//     * Inputs: [ax, ay, w]^T
//     * Outputs: [x, y, heading]^T
//     * xdot = [vx, vy, ax, ay, w]^T
//     *   = A [x, y, vx, vy, heading]^T + B [ax, ay, w]^T
//     *
//     *   Y = C x + 0 u
//     */
//    public KalmanFilter<N5, N3, N3> kalmanFilter = new KalmanFilter<>(
//            Nat.N5(), Nat.N3(),
//            new LinearSystem<>(
//                    new MatBuilder<>(Nat.N5(), Nat.N5()).fill(
//                            0,0,1,0,0,
//                            0,0,0,1,0,
//                            0,0,0,0,0,
//                            0,0,0,0,0,
//                            0,0,0,0,0
//                    ),
//                    new MatBuilder<>(Nat.N5(), Nat.N3()).fill(
//                            0,0,0,
//                            0,0,0,
//                            1,0,0,
//                            0,1,0,
//                            0,0,1
//                    ),
//                    new MatBuilder<>(Nat.N3(), Nat.N5()).fill(
//                            1,0,0,0,0,
//                            0,1,0,0,0,
//                            0,0,0,0,1
//                    ), MatrixUtils.zeros(Nat.N3(), Nat.N3())
//            ),
//            VecBuilder.fill(1.0, 1.0, 1.0, 1.0, 1.0),
//            VecBuilder.fill(0.1, 0.1, 1.0),
//            0.050);

//    public void addInput(double ax, double ay, double omega, double dt) {
//        kalmanFilter.predict(VecBuilder.fill(ax, ay, omega), dt);
//    }
//

//    AccelerometerPoseEstimator est = new AccelerometerPoseEstimator(
//            VecBuilder.fill(0.1, 0.1, 1.0, 1.0, 1.0),
//            VecBuilder.fill(0.1, 0.1, 1.0));

    // Let's just do a median filter on x/y position, because the accelerometer readings kinda suck

    // For heading we'll do a kalman filter
    // state = [position], xdot = [omega]
    // xdot = 0x + 1u
    public KalmanFilter<N1, N1, N1> headingFilter = new KalmanFilter<>(Nat.N1(),
            Nat.N1(), new LinearSystem<>(VecBuilder.fill(0), VecBuilder.fill(1), VecBuilder.fill(1), VecBuilder.fill(0)),
            VecBuilder.fill(0.1), VecBuilder.fill(0.1), 0.05);

    double lastUpdateTime = -1;

    public void update(double time, double ax, double ay, double omega) {
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
//        est.addVisionMeasurement(cameraInField, timeSeconds);
//        Log.i("PoseEst", String.format("Measurement: %s", cameraInField));
        headingFilter.correct(VecBuilder.fill(0), VecBuilder.fill(cameraInField.getRotation().getRadians()));
        Log.i("HeadingEst", String.format("Heading Estimate: %s", headingFilter.getXhat(0) * 180.0 / Math.PI));
    }

    CalculatePosePipe pipe = new CalculatePosePipe();

    public Pose2d correct(List<Marker> targetList) {
        pipe.setParams(new CalculatePosePipe.CalculatePosePipeParams(Rotation2d.fromDegrees(0)));
        CVPipe.CVPipeResult<List<Marker>> ret = pipe.run(targetList);

        for (Marker t : ret.output) {
            Pose2d maybePose = TargetConstants.getMarkerPose(t.getMarkerId());
            if (maybePose == null) continue;

            Pose2d fieldToCamera = PhotonUtils.estimateFieldToCamera(t.getCameraToTarget(), maybePose);
            correct(fieldToCamera);
            return fieldToCamera;
        }
        return null;
    }
}
