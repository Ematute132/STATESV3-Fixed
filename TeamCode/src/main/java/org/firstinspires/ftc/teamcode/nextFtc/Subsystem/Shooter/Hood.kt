package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter

import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import kotlin.math.hypot

/**
 * Hood subsystem for adjusting shooter angle based on distance.
 * Auto-updates based on robot position to goal.
 */
object Hood : Subsystem {

    // ============================================
    // HOOD POSITIONS - TUNE THESE ON FIELD!
    // ============================================
    @JvmField var DOWN = 0.0
    @JvmField var MID = 0.5
    @JvmField var FAR = 1.0

    // ============================================
    // DISTANCE THRESHOLDS - TUNE THESE ON FIELD!
    // ============================================
    @JvmField var CLOSE_THRESHOLD = 20.0
    @JvmField var MID_THRESHOLD = 40.0

    private val servo = ServoEx("hood", 1.0)

    var currentTargetPosition: Double = MID
        private set

    // Robot position suppliers - set from TeleOp before use
    var robotX: () -> Double = { 0.0 }
    var robotY: () -> Double = { 0.0 }

    var goalX: Double = 55.0
    var goalY: Double = 0.0

    // Commands - capture fresh tuned values each time
    fun cmdFar() = SetPosition(servo, FAR)
    fun cmdMid() = SetPosition(servo, MID)
    fun cmdClose() = SetPosition(servo, DOWN)

    // Auto-update flag - disable for manual override
    var autoUpdate: Boolean = true

    fun setGoalPosition(gx: Double, gy: Double) {
        goalX = gx
        goalY = gy
    }

    fun setPositionProviders(x: () -> Double, y: () -> Double) {
        robotX = x
        robotY = y
    }

    fun setPosition(newPosition: Double) {
        currentTargetPosition = newPosition.coerceIn(0.0, 1.0)
        servo.position = currentTargetPosition
    }

    fun setForDistance(distanceInches: Double): Double {
        val position = when {
            distanceInches < CLOSE_THRESHOLD -> DOWN
            distanceInches < MID_THRESHOLD -> MID
            else -> FAR
        }
        setPosition(position)
        return position
    }

    val currentPosition: Double get() = servo.position

    private fun getDistanceToGoal(): Double {
        val dx = goalX - robotX()
        val dy = goalY - robotY()
        return hypot(dx, dy)
    }

    override fun periodic() {
        // Auto-update based on distance if enabled
        if (autoUpdate) {
            val distance = getDistanceToGoal()
            setForDistance(distance)
        }

        // Telemetry
        PanelsTelemetry.telemetry.addData("Hood Auto", autoUpdate)
        PanelsTelemetry.telemetry.addData("Hood Distance", "%.1f".format(getDistanceToGoal()))
        PanelsTelemetry.telemetry.addData("Hood Position", "%.3f".format(currentPosition))
        PanelsTelemetry.telemetry.addData("Hood Target", "%.3f".format(currentTargetPosition))
    }
}
