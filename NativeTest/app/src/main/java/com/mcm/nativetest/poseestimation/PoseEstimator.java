package com.mcm.nativetest.poseestimation;

import android.util.Log;
import aruco.Marker;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpiutil.math.VecBuilder;
import org.photonvision.PhotonUtils;
import org.photonvision.vision.pipe.impl.SolvePNPPipe;

import java.util.List;
import java.util.Vector;

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

    AccelerometerPoseEstimator est = new AccelerometerPoseEstimator(
            VecBuilder.fill(0.1, 0.1, 1.0, 1.0, 1.0),
            VecBuilder.fill(0.1, 0.1, 1.0));

    public void update(double time, double ax, double ay, double omega) {
        est.updateWithTime(time, ax, ay, omega);
        Log.i("PoseEst", String.format("State Estimate: %s", est.getEstimatedPosition()));
    }

    public void correct(Pose2d cameraInField, double timeSeconds) {
        est.addVisionMeasurement(cameraInField, timeSeconds);
        Log.i("PoseEst", String.format("Measurement: %s", cameraInField));
    }

    SolvePNPPipe pipe = new SolvePNPPipe();

    public Pose2d correct(List<Marker> markerList, double timeSec) {
        // apply solvePNP math I totally didnt copy paste
        markerList = pipe.process(markerList);

        for(Marker m: markerList) {
            Pose2d maybePose = TargetConstants.getMarkerPose(m.getMarkerId());
            if(maybePose == null) continue;

            Pose2d fieldToCamera = PhotonUtils.estimateFieldToCamera(m.getCameraToTarget(), maybePose);
            correct(fieldToCamera, timeSec);
            return fieldToCamera;
        }
        return null;
    }

    public Pose2d correct(Vector<Marker> detectedMarkers) {
        return correct(detectedMarkers, System.currentTimeMillis() / 1000.0);
    }
}
