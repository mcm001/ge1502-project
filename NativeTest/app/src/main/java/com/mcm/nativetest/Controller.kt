package com.mcm.nativetest

import android.util.Log
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import kotlinx.coroutines.delay
import kotlin.math.absoluteValue
import kotlin.math.sign

class Controller(val setSpeeds: (DifferentialDriveWheelSpeeds) -> Unit, val poseSupplier: () -> Pose2d?) {

    var currentState: State = State.Nothing
        set(value) {
            field = value
            field.initialize()
        }

    suspend fun update() {
        val pose = poseSupplier() ?: return
        setSpeeds(currentState.iterate(pose))
        if(currentState.isDone()) currentState = State.Nothing
        delay(50)
    }

    fun turnToFace(pose2d: Pose2d): State {
        currentState = State.Angle(pose2d, poseSupplier)
        return currentState
    }

    sealed class State {
        abstract fun iterate(pose: Pose2d): DifferentialDriveWheelSpeeds
        open fun isDone() = false
        open fun initialize() {}

        class Angle(private val targetPose: Pose2d, private val poseSupplier: () -> Pose2d?) : State() {
            private val pid = PIDController(0.3, 0.0, 0.0)
                .apply {
                    enableContinuousInput(0.0, 360.0)
                }

            override fun initialize() {
                pid.reset()
            }

            override fun iterate(pose: Pose2d): DifferentialDriveWheelSpeeds {
                val currentPose = poseSupplier() ?: return DifferentialDriveWheelSpeeds()
                val goalInCamera = targetPose.relativeTo(currentPose)
                val yaw = goalInCamera.translation.toRotation2d() + currentPose.rotation
                Log.i("Controller", "Turning to face yaw $yaw")

                var power = pid.calculate(pose.rotation.degrees, yaw.degrees)
                power += 0.2 * sign(power) // Static friction
                println(power)

                return DifferentialDriveWheelSpeeds(-power, power)
            }

            override fun isDone() = pid.positionError.absoluteValue < 1
        }

        class Distance(distInches: Double) : State() {
            // hacky distance -> speed conversion
            private val timePerInch = 0.5

            var firstRun = true
            private var startTime: Long = -1
            private val duration = distInches * timePerInch

            override fun iterate(pose: Pose2d): DifferentialDriveWheelSpeeds {
                if(firstRun) startTime = System.currentTimeMillis()
                return DifferentialDriveWheelSpeeds(0.6, 0.5)
            }

            override fun isDone(): Boolean {
                return startTime > 0 && (System.currentTimeMillis() - startTime) > duration * 1000
            }
        }

        class Power(val left: Double, val right: Double) : State() {
            override fun iterate(pose: Pose2d) = DifferentialDriveWheelSpeeds(left, right)
        }

        object Nothing : State() {
            override fun iterate(pose: Pose2d): DifferentialDriveWheelSpeeds {
                return DifferentialDriveWheelSpeeds()
            }
        }

    }
}

private fun Translation2d.toRotation2d() = Rotation2d(x, y)


