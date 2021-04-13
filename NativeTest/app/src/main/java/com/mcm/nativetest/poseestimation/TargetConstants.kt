package com.mcm.nativetest.poseestimation

import edu.wpi.first.wpilibj.geometry.Pose2d
import com.mcm.nativetest.poseestimation.TargetConstants
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpiutil.math.Pair
import java.util.HashMap

private val Number.degrees get() = Rotation2d.fromDegrees(toDouble())

object TargetConstants {
    @JvmStatic
    fun getMarkerPose(code: Int): Pose2d? {
        return targetMap[code]
    }

    private val targetMap = hashMapOf(
        1 to Pose2d(0.0, 0.06, 0.degrees),
        2 to Pose2d(0.0, -0.06, 0.degrees)
    )
}