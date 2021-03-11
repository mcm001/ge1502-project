package com.mcm.nativetest;

import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.MatrixUtils;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N5;

public class PoseEstimator {

    /** States: [x, y, vx, vy, heading]^T
     * Inputs: [ax, ay, w]^T
     * Outputs: [x, y, heading]^T
     * xdot = [vx, vy, ax, ay, w]^T
     *   = A [x, y, vx, vy, heading]^T + B [ax, ay, w]^T
     *
     *   Y = C x + 0 u
     */
    public KalmanFilter<N5, N3, N3> kalmanFilter = new KalmanFilter<>(
            Nat.N5(), Nat.N3(),
            new LinearSystem<>(
                    new MatBuilder<>(Nat.N5(), Nat.N5()).fill(
                            0,0,1,0,0,
                            0,0,0,1,0,
                            0,0,0,0,0,
                            0,0,0,0,0,
                            0,0,0,0,0
                    ),
                    new MatBuilder<>(Nat.N5(), Nat.N3()).fill(
                            0,0,0,
                            0,0,0,
                            1,0,0,
                            0,1,0,
                            0,0,1
                    ),
                    new MatBuilder<>(Nat.N3(), Nat.N5()).fill(
                            1,0,0,0,0,
                            0,1,0,0,0,
                            0,0,0,0,1
                    ), MatrixUtils.zeros(Nat.N3(), Nat.N3())
            ),
            VecBuilder.fill(1.0, 1.0, 1.0, 1.0, 1.0),
            VecBuilder.fill(0.1, 0.1, 1.0),
            0.050);

    public void addInput(double ax, double ay, double omega, double dt) {
        kalmanFilter.predict(VecBuilder.fill(ax, ay, omega), dt);
    }

    public void correct(Pose2d cameraInField) {

    }
}
