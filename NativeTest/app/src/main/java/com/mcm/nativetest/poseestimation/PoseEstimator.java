package com.mcm.nativetest.poseestimation;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class PoseEstimator {

    long lastUpdateTime = -1;

    public Rotation2d heading = new Rotation2d();

    public void predict(double time, double ax, double ay, double omega) {
        long now = System.currentTimeMillis();
        long dt = now - lastUpdateTime;
        if (lastUpdateTime < 0) {
            lastUpdateTime = System.currentTimeMillis();
            dt = 50L;
        }
        dt /= 1000.0;
        lastUpdateTime = now;

        heading = heading.plus(new Rotation2d(omega * dt));
    }

    public void reset() {
        heading = new Rotation2d();
    }
}
