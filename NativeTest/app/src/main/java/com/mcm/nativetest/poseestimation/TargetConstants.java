package com.mcm.nativetest.poseestimation;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpiutil.math.Pair;

import java.util.HashMap;

public final class TargetConstants {

    public static Pose2d getMarkerPose(int code) {
        return targetMap.get(code);
    }

    private static final HashMap<Integer, Pose2d> targetMap = createMap(
            Pair.of(1, new Pose2d())
    );

    private static HashMap<Integer, Pose2d> createMap(Pair<Integer, Pose2d>... pairs) {
        HashMap<Integer, Pose2d> map = new HashMap<>();
        for(Pair<Integer, Pose2d> pair: pairs) map.put(pair.getFirst(), pair.getSecond());
        return map;
    }
}
