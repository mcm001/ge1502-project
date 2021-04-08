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

import android.graphics.Color;
import org.apache.commons.lang3.tuple.Pair;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.photonvision.common.logging.LogGroup;
import org.photonvision.common.logging.Logger;
import org.photonvision.common.util.ColorHelper;
import org.photonvision.vision.calibration.CameraCalibrationCoefficients;
import org.photonvision.vision.frame.FrameDivisor;
import org.photonvision.vision.pipe.MutatingPipe;
import org.photonvision.vision.target.TargetModel;
import org.photonvision.vision.target.TrackedTarget;

import java.util.Arrays;
import java.util.List;

public class Draw3dTargetsPipe
        extends MutatingPipe<Pair<Mat, List<TrackedTarget>>, Draw3dTargetsPipe.Draw3dContoursParams> {

    Logger logger = new Logger(Draw3dTargetsPipe.class, LogGroup.VisionModule);

    @Override
    protected Void process(Pair<Mat, List<TrackedTarget>> in) {
        if (!params.shouldDraw) return null;

        for (TrackedTarget target : in.getRight()) {

            // draw convex hull
            MatOfPoint pointMat = new MatOfPoint();
            divideMat2f(target.m_mainContour.getConvexHull(), pointMat);
            if (pointMat.size().empty()) {
                logger.error("Convex hull is empty?");
                logger.debug(
                        "Orig. Convex Hull: " + target.m_mainContour.getConvexHull().size().toString());
                continue;
            }
            Imgproc.drawContours(
                    in.getLeft(), Arrays.asList(pointMat), -1, ColorHelper.colorToScalar(Color.GREEN), 1);

            // draw approximate polygon
            MatOfPoint2f poly = target.getApproximateBoundingPolygon();
            if (poly != null) {
                divideMat2f(poly, pointMat);
                Imgproc.drawContours(
                        in.getLeft(), Arrays.asList(pointMat), -1, ColorHelper.colorToScalar(Color.BLUE), 2);
            }

            // Draw floor and top
            if (target.getCameraRelativeRvec() != null && target.getCameraRelativeTvec() != null) {
                MatOfPoint2f tempMat = new MatOfPoint2f();
                Mat jac = new Mat();
                MatOfPoint3f bottomModel = params.targetModel.getVisualizationBoxBottom();
                MatOfPoint3f topModel = params.targetModel.getVisualizationBoxTop();
                Calib3d.projectPoints(
                        bottomModel,
                        target.getCameraRelativeRvec(),
                        target.getCameraRelativeTvec(),
                        params.cameraCalibrationCoefficients.getCameraIntrinsicsMat(),
                        params.cameraCalibrationCoefficients.getCameraExtrinsicsMat(),
                        tempMat,
                        jac);
                List<Point> bottomPoints = tempMat.toList();
                Calib3d.projectPoints(
                        topModel,
                        target.getCameraRelativeRvec(),
                        target.getCameraRelativeTvec(),
                        params.cameraCalibrationCoefficients.getCameraIntrinsicsMat(),
                        params.cameraCalibrationCoefficients.getCameraExtrinsicsMat(),
                        tempMat,
                        jac);
                List<Point> topPoints = tempMat.toList();

                dividePointList(bottomPoints);
                dividePointList(topPoints);

                // floor, then pillers, then top
                for (int i = 0; i < bottomPoints.size(); i++) {
                    Imgproc.line(
                            in.getLeft(),
                            bottomPoints.get(i),
                            bottomPoints.get((i + 1) % (bottomPoints.size())),
                            ColorHelper.colorToScalar(Color.GREEN),
                            3);
                }
                for (int i = 0; i < bottomPoints.size(); i++) {
                    Imgproc.line(
                            in.getLeft(),
                            bottomPoints.get(i),
                            topPoints.get(i),
                            ColorHelper.colorToScalar(Color.BLUE),
                            3);
                }
                for (int i = 0; i < topPoints.size(); i++) {
                    Imgproc.line(
                            in.getLeft(),
                            topPoints.get(i),
                            topPoints.get((i + 1) % (bottomPoints.size())),
                            ColorHelper.colorToScalar(Color.RED),
                            3);
                }
                tempMat.release();
                jac.release();
            }
            pointMat.release();

            // draw corners
            List<Point> corners = target.getTargetCorners();
            if (corners != null && !corners.isEmpty()) {
                for (Point corner : corners) {
                    double x = corner.x / (double) params.divisor.value;
                    double y = corner.y / (double) params.divisor.value;

                    Imgproc.circle(
                            in.getLeft(),
                            new Point(x, y),
                            params.radius,
                            ColorHelper.colorToScalar(params.color),
                            params.radius);
                }
            }
        }

        return null;
    }

    private void divideMat2f(MatOfPoint2f src, MatOfPoint dst) {
        Point[] hull = src.toArray();
        Point[] pointArray = new Point[hull.length];
        for (int i = 0; i < hull.length; i++) {
            Point hullAtI = hull[i];
            pointArray[i] =
                    new Point(
                            hullAtI.x / (double) params.divisor.value, hullAtI.y / (double) params.divisor.value);
        }
        dst.fromArray(pointArray);
    }

    /** Scale a given point list by the current frame divisor. the point list is mutated! */
    private void dividePointList(List<Point> points) {
        for (Point p : points) {
            p.x = p.x / (double) params.divisor.value;
            p.y = p.y / (double) params.divisor.value;
        }
    }

    public static class Draw3dContoursParams {
        public int radius = 2;
        public int color = Color.RED;

        public final boolean shouldDraw;
        public final TargetModel targetModel;
        public final CameraCalibrationCoefficients cameraCalibrationCoefficients;
        public final FrameDivisor divisor;

        public Draw3dContoursParams(
                boolean shouldDraw,
                CameraCalibrationCoefficients cameraCalibrationCoefficients,
                TargetModel targetModel,
                FrameDivisor divisor) {
            this.shouldDraw = shouldDraw;
            this.cameraCalibrationCoefficients = cameraCalibrationCoefficients;
            this.targetModel = targetModel;
            this.divisor = divisor;
        }
    }
}
