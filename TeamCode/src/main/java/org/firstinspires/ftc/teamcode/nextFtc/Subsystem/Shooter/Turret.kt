package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.*

/**
 * Turret subsystem - adapted from working BURNED code
 * Supports: IDLE, MANUAL, ODOMETRY, KALMAN_AIM states
 */
object Turret : Subsystem {

    enum class State { IDLE, MANUAL, ODOMETRY, KALMAN_AIM }

    var motor = MotorEx("turret")

    // Control system - using BURNED's working PID values
    var controller = controlSystem {
        posPid(0.3, 0.0, 0.05)  // kP, kI, kD - these work!
        basicFF(0.25, 0.0, 0.0)  // kV, kA, kS
    }

    var manualPower = 0.0
    var currentState = State.IDLE
        internal set

    // Keep track of last command for cancellation
    internal var lastCommand: Command? = null

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

    // Internal target for aim commands
    private var targetAngle: Angle = 0.0.rad
    private var targetVelocity: Double = 0.0

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
            State.KALMAN_AIM -> {
                // Kalman filter runs in TripleFusionAim command, this state just applies control
                applyTargetControl()
            }
        }

        // Telemetry
        PanelsTelemetry.telemetry.addData("Turret State", currentState.name)
        PanelsTelemetry.telemetry.addData("Turret Yaw (deg)", "%.1f".format(Math.toDegrees(turretYaw)))
        PanelsTelemetry.telemetry.addData("Turret Target", "%.1f".format(Math.toDegrees(targetAngle.inRad)))
        PanelsTelemetry.telemetry.addData("Motor Power", "%.2f".format(motor.power))
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

    /**
     * Apply control using targetAngle and targetVelocity (set by fusion command)
     */
    private fun applyTargetControl() {
        val currentYaw = getYaw()

        controller.goal = KineticState(
            angleToEncoder(targetAngle.inRad),
            targetVelocity
        )

        var power = controller.calculate(motor.state)

        val errorDeg = Math.toDegrees(abs(targetAngle.inRad - currentYaw))
        if (errorDeg > 0.5) {
            power += (if (power >= 0) 1.0 else -1.0) * minPower
        } else {
            if (abs(targetVelocity) < 0.1) power = 0.0
        }

        motor.power = power.coerceIn(-maxPower, maxPower)
    }

    private fun angleToEncoder(angleRad: Double): Double {
        // Simple conversion - adjust based on your encoder calibration
        return angleRad / RADIANS_PER_TICK
    }

    private fun aimWithOdometry() {
        val pose = PedroComponent.follower.pose
        val currentX = pose.x
        val currentY = pose.y
        val currentHeading = pose.heading.inRad

        val deltaX = goalX - currentX
        val deltaY = goalY - currentY
        val fieldAngle = atan2(deltaY, deltaX)

        val normalizedHeading = if (abs(currentHeading) > 2.0 * PI) Math.toRadians(currentHeading) else currentHeading

        val target = normalizeAngle(fieldAngle - normalizedHeading)
        
        // Set target for control
        targetAngle = target
        targetVelocity = -robotAngularVelocity * kV
        
        applyTargetControl()
    }

    /**
     * Set target angle (used by Kalman fusion commands)
     */
    fun setTargetAngle(angle: Angle, compensateVelocity: Boolean = true) {
        targetAngle = clampAngle(angle)
        targetVelocity = if (compensateVelocity) -robotAngularVelocity * kV else 0.0
    }

    fun getYaw(): Double = normalizeAngle(motor.currentPosition * RADIANS_PER_TICK)

    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }

    fun clampAngle(angle: Angle): Angle {
        return angle.inRad.coerceIn(MIN_ANGLE, MAX_ANGLE).rad
    }

    fun setGoalPosition(x: Double, y: Double) {
        goalX = x
        goalY = y
    }

    fun aimWithOdometry() { currentState = State.ODOMETRY }
    
    fun aimWithKalman() { currentState = State.KALMAN_AIM }
    
    fun stop() { 
        currentState = State.IDLE
        motor.power = 0.0
        if (lastCommand != null) {
            CommandManager.cancelCommand(lastCommand!!)
            lastCommand = null
        }
    }

    /**
     * Register command for cancellation tracking
     */
    fun registerCommand(command: Command) {
        if (lastCommand != null && lastCommand != command) {
            CommandManager.cancelCommand(lastCommand!!)
        }
        lastCommand = command
    }

    // Manual control function
    fun Manual(power: Double) {
        manualPower = power
        currentState = State.MANUAL
    }
}
