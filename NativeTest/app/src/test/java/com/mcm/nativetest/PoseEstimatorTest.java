package com.mcm.nativetest;

import com.mcm.nativetest.poseestimation.AccelerometerPoseEstimator;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N5;
import org.junit.Test;

public class PoseEstimatorTest {

    @Test
    public void testPredict() {
        AccelerometerPoseEstimator est = new AccelerometerPoseEstimator(
                VecBuilder.fill(0.1, 0.1, 1.0, 1.0, 1.0),
                VecBuilder.fill(0.1, 0.1, 1.0));

        // x y vx vy theta
        Matrix<N5, N1> groundTruth = VecBuilder.fill(0, 0, 0, 0, 0);

        // First we verify the system
        // u = [ax, ay, omega]^T
        Matrix<N3, N1> u = VecBuilder.fill(1, 0, 0);
        for(int i = 0; i < 50; i++) {
            // 1 second
            System.out.println(est.m_observer.getXhat());
            est.m_observer.predict(u, 0.020);
        }
    }

}
