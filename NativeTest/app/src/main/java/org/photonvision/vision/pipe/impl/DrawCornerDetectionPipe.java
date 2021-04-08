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

import org.apache.commons.lang3.tuple.Pair;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.photonvision.vision.pipe.MutatingPipe;
import org.photonvision.vision.target.TrackedTarget;

import java.util.List;

public class DrawCornerDetectionPipe
        extends MutatingPipe<Pair<Mat, List<TrackedTarget>>, DrawCornerDetectionPipe.DrawCornerParams> {

    @Override
    protected Void process(Pair<Mat, List<TrackedTarget>> in) {
        Mat image = in.getLeft();

        for (TrackedTarget target : in.getRight()) {
            List<Point> corners = target.getTargetCorners();
            for (Point corner : corners) {
                Imgproc.circle(image, corner, params.dotRadius, params.dotColor);
            }
        }

        return null;
    }

    public static class DrawCornerParams {
        int dotRadius;
        Scalar dotColor;
    }
}
