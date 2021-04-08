/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package org.photonvision.vision.pipe.impl;

import aruco.Marker;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.photonvision.common.logging.LogGroup;
import org.photonvision.common.logging.Logger;
import org.photonvision.vision.pipe.CVPipe;

import java.util.List;

public class CalculatePosePipe
        extends CVPipe<List<Marker>, List<Marker>, CalculatePosePipe.CalculatePosePipeParams> {

    private static final Logger logger = new Logger(CalculatePosePipe.class, LogGroup.VisionModule);

    private final MatOfPoint2f imagePoints = new MatOfPoint2f();

    private boolean hasWarned = false;

    @Override
    protected List<Marker> process(List<Marker> targetList) {
        for (Marker target : targetList) {
            calculateTargetPose(target);
        }
        return targetList;
    }

    private void calculateTargetPose(Marker target) {
        Transform2d targetPose = correctLocationForCameraPitch(
                target.getTvec(), target.getRvec(), params.cameraPitchAngle);

        target.setCameraToTarget(targetPose);
    }

    Mat rotationMatrix = new Mat();
    Mat inverseRotationMatrix = new Mat();
    Mat pzeroWorld = new Mat();
    Mat kMat = new Mat();
    Mat scaledTvec;

    @SuppressWarnings("DuplicatedCode") // yes I know we have another solvePNP pipe
    private Transform2d correctLocationForCameraPitch(
            Mat tVec, Mat rVec, Rotation2d cameraPitchAngle) {
        // Algorithm from team 5190 Green Hope Falcons. Can also be found in Ligerbot's vision
        // whitepaper
        double tiltAngle = cameraPitchAngle.getRadians();

        // the left/right distance to the target, unchanged by tilt.
        double x = tVec.get(0, 0)[0];

        // Z distance in the flat plane is given by
        // Z_field = z cos theta + y sin theta.
        // Z is the distance "out" of the camera (straight forward).
        double zField = tVec.get(2, 0)[0] * Math.cos(tiltAngle) + tVec.get(1, 0)[0] * Math.sin(tiltAngle);

        Calib3d.Rodrigues(rVec, rotationMatrix);
        Core.transpose(rotationMatrix, inverseRotationMatrix);

        scaledTvec = matScale(tVec, -1);

        Core.gemm(inverseRotationMatrix, scaledTvec, 1, kMat, 0, pzeroWorld);
        scaledTvec.release();

        double angle2 = Math.atan2(pzeroWorld.get(0, 0)[0], pzeroWorld.get(2, 0)[0]);

        // target rotation is the rotation of the target relative to straight ahead. this number
        // should be unchanged if the robot purely translated left/right.
        double targetRotation = -angle2; // radians

        // We want a vector that is X forward and Y left.
        // We have a Z_field (out of the camera projected onto the field), and an X left/right.
        // so Z_field becomes X, and X becomes Y

        Translation2d targetLocation = new Translation2d(zField, -x);
        return new Transform2d(targetLocation, new Rotation2d(targetRotation));
    }

    /**
     * Element-wise scale a matrix by a given factor
     *
     * @param src    the source matrix
     * @param factor by how much to scale each element
     * @return the scaled matrix
     */
    @SuppressWarnings("SameParameterValue")
    private static Mat matScale(Mat src, double factor) {
        Mat dst = new Mat(src.rows(), src.cols(), src.type());
        Scalar s = new Scalar(factor);
        Core.multiply(src, s, dst);
        return dst;
    }

    public static class CalculatePosePipeParams {
        private final Rotation2d cameraPitchAngle;

        public CalculatePosePipeParams(
                Rotation2d cameraPitchAngle) {
            this.cameraPitchAngle = cameraPitchAngle;
        }
    }
}
