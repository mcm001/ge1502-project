package com.mcm.nativetest;

import org.junit.Test;

public class PoseEstimatorTest {

    @Test
    public void testPredict() {
        PoseEstimator estimator = new PoseEstimator();
        for(int i = 0; i < 10; i++) {
            estimator.addInput(1, 0, 0, 0.020);
        }
        System.out.println(estimator.kalmanFilter.getXhat());
    }

}
