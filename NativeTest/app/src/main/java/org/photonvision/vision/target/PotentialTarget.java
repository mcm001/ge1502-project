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

package org.photonvision.vision.target;

import org.opencv.core.RotatedRect;
import org.photonvision.vision.Releasable;
import org.photonvision.vision.opencv.CVShape;
import org.photonvision.vision.opencv.Contour;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class PotentialTarget implements Releasable {

    public final Contour m_mainContour;
    public final List<Contour> m_subContours;
    public final CVShape shape;

    public PotentialTarget(Contour inputContour) {
        this(inputContour, Collections.emptyList());
    }

    public PotentialTarget(Contour inputContour, List<Contour> subContours) {
        this(inputContour, subContours, null);
    }

    public PotentialTarget(Contour inputContour, List<Contour> subContours, CVShape shape) {
        m_mainContour = inputContour;
        m_subContours = new ArrayList<>(subContours);
        this.shape = shape;
    }

    public PotentialTarget(Contour inputContour, CVShape shape) {
        this(inputContour, Arrays.asList(), shape);
    }

    public RotatedRect getMinAreaRect() {
        return m_mainContour.getMinAreaRect();
    }

    public double getArea() {
        return m_mainContour.getArea();
    }

    @Override
    public void release() {
        m_mainContour.release();
        for (Contour sc : m_subContours) {
            sc.release();
        }
        m_subContours.clear();
        if (shape != null) shape.release();
    }
}
