package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.*

/**
 * Turret subsystem - adapted from working BURNED code
 */
object Turret : Subsystem {

    enum class State { IDLE, MANUAL, ODOMETRY }

    var motor = MotorEx("turret")

    // Control system - using BURNED's working PID values
    var controller = controlSystem {
        posPid(0.3, 0.0, 0.05)  // kP, kI, kD - these work!
        basicFF(0.25, 0.0, 0.0)  // kV, kA, kS
    }

    var manualPower = 0.0
    var currentState = State.IDLE

    // Goal positions - adjust for your field!
    @JvmField var goalX = 144.0  // Red goal
    @JvmField var goalY = 144.0  // Field center Y

    @JvmField var minPower: Double = 0.15
    @JvmField var maxPower: Double = 0.75
    @JvmField var alignmentTolerance: Double = 2.0
    @JvmField var kV: Double = 0.25  // Velocity compensation

    const val GEAR_RATIO = 3.62068965517  // 105/29
    const val MOTOR_TICKS_PER_REV = 537.7
    private const val RADIANS_PER_TICK = 2.0 * PI / (MOTOR_TICKS_PER_REV * GEAR_RATIO)

    const val MIN_ANGLE = -3.0 * PI / 4.0  // -135 degrees
    const val MAX_ANGLE = 3.0 * PI / 4.0   // +135 degrees

    // State tracking
    private val velTimer = ElapsedTime()
    private var lastRobotHeading = 0.0
    var robotAngularVelocity = 0.0
        private set

    var turretYaw: Double = 0.0

    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        velTimer.reset()
    }

    override fun periodic() {
        turretYaw = getYaw()
        updateRobotVelocity()

        when (currentState) {
            State.IDLE -> motor.power = 0.0
            State.MANUAL -> motor.power = manualPower.coerceIn(-maxPower, maxPower)
            State.ODOMETRY -> aimWithOdometry()
        }

        // Telemetry
        PanelsTelemetry.telemetry.addData("Turret State", currentState.name)
        PanelsTelemetry.telemetry.addData("Turret Yaw (deg)", Math.toDegrees(turretYaw))
        PanelsTelemetry.telemetry.addData("Motor Power", motor.power)
    }

    private fun updateRobotVelocity() {
        val dt = velTimer.seconds()
        if (dt > 0.001) {
            val currentHeading = PedroComponent.follower.pose.heading.inRad
            val deltaHeading = normalizeAngle(currentHeading - lastRobotHeading)
            robotAngularVelocity = deltaHeading / dt
            lastRobotHeading = currentHeading
            velTimer.reset()
        }
    }

    private fun applyControl(targetYaw: Double, targetVelocity: Double = 0.0) {
        val clampedTarget = targetYaw.coerceIn(MIN_ANGLE, MAX_ANGLE)
        val currentYaw = getYaw()

        controller.goal = KineticState(clampedTarget, targetVelocity)

        var power = controller.calculate(KineticState(currentYaw, 0.0))

        val errorDeg = Math.toDegrees(abs(clampedTarget - currentYaw))
        if (errorDeg > 0.5) {
            power += (if (power >= 0) 1.0 else -1.0) * minPower
        } else {
            if (abs(targetVelocity) < 0.1) power = 0.0
        }

        motor.power = power.coerceIn(-maxPower, maxPower)
    }

    fun aimWithOdometry() {
        val pose = PedroComponent.follower.pose
        val currentX = pose.x
        val currentY = pose.y
        val currentHeading = pose.heading.inRad

        val deltaX = goalX - currentX
        val deltaY = goalY - currentY
        val fieldAngle = atan2(deltaY, deltaX)

        val normalizedHeading = if (abs(currentHeading) > 2.0 * PI) Math.toRadians(currentHeading) else currentHeading

        applyControl(normalizeAngle(fieldAngle - normalizedHeading), -robotAngularVelocity * kV)
    }

    fun getYaw(): Double = normalizeAngle(motor.currentPosition * RADIANS_PER_TICK)

    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }

    fun setGoalPosition(x: Double, y: Double) {
        goalX = x
        goalY = y
    }

    fun aimWithOdometry() { currentState = State.ODOMETRY }
    fun stop() { currentState = State.IDLE; motor.power = 0.0 }

    // Manual control class
    fun Manual(power: Double) {
        manualPower = power
        currentState = State.MANUAL
    }
}
