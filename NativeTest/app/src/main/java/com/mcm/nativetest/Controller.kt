package com.mcm.nativetest

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import kotlinx.coroutines.delay

class Controller(val setSpeeds: (DifferentialDriveWheelSpeeds) -> Unit) {

    var currentState: State = State.Nothing

    suspend fun update() {
        setSpeeds(currentState.iterate())
        delay(50)
    }

    sealed class State {
        abstract fun iterate(): DifferentialDriveWheelSpeeds

        class Spinning(targetAngle: Rotation2d) : State() {
            override fun iterate(): DifferentialDriveWheelSpeeds {
                TODO("Not yet implemented")
            }
        }
        class Distance(targetDistance: Double) : State() {
            override fun iterate(): DifferentialDriveWheelSpeeds {
                TODO("Not yet implemented")
            }
        }
        class Power(val left: Double, val right: Double) : State() {
            override fun iterate() = DifferentialDriveWheelSpeeds(left, right)
        }

        object Nothing : State() {
            override fun iterate(): DifferentialDriveWheelSpeeds {
                return DifferentialDriveWheelSpeeds()
            }
        }
    }
}