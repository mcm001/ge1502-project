// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.mcm.nativetest.poseestimation;

import edu.wpi.first.wpilibj.estimator.AngleStatistics;
import edu.wpi.first.wpilibj.estimator.KalmanFilterLatencyCompensator;
import edu.wpi.first.wpilibj.estimator.UnscentedKalmanFilter;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.math.Discretization;
import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.*;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N5;

import java.util.function.BiConsumer;

/**
 * This class wraps an {@link UnscentedKalmanFilter Unscented Kalman Filter} to fuse
 * latency-compensated vision measurements with mecanum drive encoder velocity measurements. It will
 * correct for noisy measurements and encoder drift. It is intended to be an easy but more accurate
 * drop-in for {@link }.
 *
 * <p>Our state-space system is:
 *
 * <p><strong> x = [[x, y, theta]]^T </strong> in the field-coordinate system.
 *
 * <p><strong> u = [[vx, vy, theta]]^T </strong> in the field-coordinate system.
 *
 * <p><strong> y = [[x, y, theta]]^T </strong> in field coords from vision, or <strong> y =
 * [[theta]]^T </strong> from the gyro.
 */
public class AccelerometerPoseEstimator {
    public final UnscentedKalmanFilter<N5, N3, N3> m_observer;
    private final BiConsumer<Matrix<N3, N1>, Matrix<N3, N1>> m_visionCorrect;
    private final KalmanFilterLatencyCompensator<N5, N3, N3> m_latencyCompensator;

    private final double m_nominalDt; // Seconds
    private double m_prevTimeSeconds = -1.0;

    private Matrix<N3, N3> m_visionDiscreteR;

    /**
     * Constructs a MecanumDrivePoseEstimator.
     *
     * @param stateStdDevs             Standard deviations of model states. Increase these numbers to trust your
     *                                 model's state estimates less. This matrix is in the form [x, y, theta]^T, with units in
     *                                 meters and radians.
     */
    public AccelerometerPoseEstimator(
            Matrix<N5, N1> stateStdDevs,
            Matrix<N3, N1> measurementStdDevs) {
        this(
                stateStdDevs,
                measurementStdDevs,
                0.02);
    }

    /**
     * Constructs a MecanumDrivePoseEstimator.
     *
     * @param stateStdDevs             Standard deviations of model states. Increase these numbers to trust your
     *                                 model's state estimates less. This matrix is in the form [x, y, theta]^T, with units in
     *                                 meters and radians.
     * @param measurementStdDevs       Standard deviations of the vision measurements. Increase these
     *                                 numbers to trust global measurements from vision less. This matrix is in the form [x, y,
     *                                 theta]^T, with units in meters and radians.
     * @param nominalDtSeconds         The time in seconds between each robot loop.
     */
    @SuppressWarnings("ParameterName")
    public AccelerometerPoseEstimator(
            Matrix<N5, N1> stateStdDevs,
            Matrix<N3, N1> measurementStdDevs,
            double nominalDtSeconds) {
        m_nominalDt = nominalDtSeconds;


    /** States: [x, y, heading, vx, vy]^T
     * Inputs: [ax, ay, w]^T
     * Outputs: [x, y, heading]^T
     * xdot = [vx, vy, w, ax, ay]^T
     *   = A [x, y, heading, vx, vy]^T + B [ax, ay, w]^T
     *
     *   Y = C x + 0 u
     */

        LinearSystem<N5, N3, N3> system =
                new LinearSystem<>(
                        new MatBuilder<>(Nat.N5(), Nat.N5()).fill(
                                0, 0, 0, 1, 0,
                                0, 0, 0, 0, 1,
                                0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0
                        ),
                        new MatBuilder<>(Nat.N5(), Nat.N3()).fill(
                                0, 0, 0,
                                0, 0, 0,
                                0, 0, 1,
                                1, 0, 0,
                                0, 1, 0
                        ),
                        new MatBuilder<>(Nat.N3(), Nat.N5()).fill(
                                1, 0, 0, 0, 0,
                                0, 1, 0, 0, 0,
                                0, 0, 1, 0, 0
                        ), MatrixUtils.zeros(Nat.N3(), Nat.N3()));

        m_observer =
                new UnscentedKalmanFilter<>(
                        Nat.N5(),
                        Nat.N3(),
                        // f(x, u) = Ax + Bu
                        (x, u) -> system.getA().times(x).plus(system.getB().times(u)),
                        // h(x, u) -> y
                        system::calculateY,
                        stateStdDevs,
                        measurementStdDevs,
                        AngleStatistics.angleMean(2),
                        AngleStatistics.angleMean(0),
                        AngleStatistics.angleResidual(2),
                        AngleStatistics.angleResidual(0),
                        AngleStatistics.angleAdd(2),
                        m_nominalDt);
        m_latencyCompensator = new KalmanFilterLatencyCompensator<>();

        // Initialize vision R
        setVisionMeasurementStdDevs(measurementStdDevs);

        m_visionCorrect =
                (Matrix<N3, N1> u, Matrix<N3, N1> y) ->
                        m_observer.correct(
                                Nat.N3(),
                                u,
                                y,
                                system::calculateY,
                                m_visionDiscreteR,
                                AngleStatistics.angleMean(2),
                                AngleStatistics.angleResidual(2),
                                AngleStatistics.angleResidual(2),
                                AngleStatistics.angleAdd(2));

        m_observer.setXhat(MatrixUtils.zeros(Nat.N5()));
    }

    /**
     * Sets the pose estimator's trust of global measurements. This might be used to change trust in
     * vision measurements after the autonomous period, or to change trust as distance to a vision
     * target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
     *                                 numbers to trust global measurements from vision less. This matrix is in the form [x, y,
     *                                 theta]^T, with units in meters and radians.
     */
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        Matrix<N3, N3> visionContR = StateSpaceUtil.makeCovarianceMatrix(Nat.N3(), visionMeasurementStdDevs);
        m_visionDiscreteR = Discretization.discretizeR(visionContR, m_nominalDt);
    }

    /**
     * Resets the robot's position on the field.
     *
     * <p>You NEED to reset your encoders (to zero) when calling this method.
     *
     * <p>The gyroscope angle does not need to be reset in the user's robot code. The library
     * automatically takes care of offsetting the gyro angle.
     *
     * @param poseMeters The position on the field that your robot is at.
     */
    public void resetPosition(Pose2d poseMeters) {
        m_observer.setXhat(StateSpaceUtil.poseTo5dVector(poseMeters));
    }

    /**
     * Gets the pose of the robot at the current time as estimated by the Unscented Kalman Filter.
     *
     * @return The estimated robot pose in meters.
     */
    public Pose2d getEstimatedPosition() {
        return new Pose2d(
                m_observer.getXhat(0), m_observer.getXhat(1), new Rotation2d(m_observer.getXhat(2)));
    }

    /**
     * Add a vision measurement to the Unscented Kalman Filter. This will correct the odometry pose
     * estimate while still accounting for measurement noise.
     *
     * <p>This method can be called as infrequently as you want, as long as you are calling {@link
     * AccelerometerPoseEstimator#updateWithTime} every loop.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds      The timestamp of the vision measurement in seconds. Note that if you
     *                              don't use your own time source by calling {@link AccelerometerPoseEstimator#updateWithTime}
     *                              then you must use a timestamp with an epoch since FPGA startup (i.e. the epoch of this
     *                              timestamp is the same epoch as Timer.getFPGATimestamp.) This means that you should use
     *                              Timer.getFPGATimestamp as your time source or sync the epochs.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        m_latencyCompensator.applyPastGlobalMeasurement(
                Nat.N3(),
                m_observer,
                m_nominalDt,
                StateSpaceUtil.poseTo3dVector(visionRobotPoseMeters),
                m_visionCorrect,
                timestampSeconds);
    }

//    /**
//     * Updates the the Unscented Kalman Filter using only wheel encoder information. This should be
//     * called every loop, and the correct loop period must be passed into the constructor of this
//     * class.
//     *
//     * @return The estimated pose of the robot in meters.
//     */
//    public Pose2d update(double ax, double ay, double omega) {
//        return updateWithTime(WPIUtilJNI.now() * 1.0e-6, ax, ay, omega);
//    }

    /**
     * Updates the the Unscented Kalman Filter using only wheel encoder information. This should be
     * called every loop, and the correct loop period must be passed into the constructor of this
     * class.
     *
     * @param currentTimeSeconds Time at which this method was called, in seconds.
     * @return The estimated pose of the robot in meters.
     */
    @SuppressWarnings("LocalVariableName")
    public Pose2d updateWithTime(
            double currentTimeSeconds, double ax, double ay, double omega) {
        double dt = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : m_nominalDt;
        m_prevTimeSeconds = currentTimeSeconds;

        Translation2d fieldRelativeAccel =
                new Translation2d(ax, ay).rotateBy(getEstimatedPosition().getRotation());

        Vector<N3> u = VecBuilder.fill(fieldRelativeAccel.getX(), fieldRelativeAccel.getY(), omega);

        m_latencyCompensator.addObserverState(m_observer, u, null, currentTimeSeconds);
        m_observer.predict(u, dt);

        return getEstimatedPosition();
    }
}
