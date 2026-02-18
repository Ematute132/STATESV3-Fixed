package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.abs

/**
 * Base turret subsystem.
 * Works with OdometryAim, VisionAim, and Kalman fusion commands.
 */
object Turret : Subsystem {

    enum class State { IDLE, MANUAL, AIMING }

    // ============================================
    // HARDWARE - UPDATE THESE FOR YOUR ROBOT
    // ============================================
    var motor = MotorEx("turret")  // TODO: Update motor name

    // ============================================
    // PHYSICAL CONSTANTS - MEASURE YOUR ROBOT
    // ============================================
    const val GEAR_RATIO = 3.62           // TODO: Verify - Motor:Turret ratio
    const val MOTOR_TICKS = 537.7         // Neverest Orbital 60
    private const val TICKS_PER_RADIAN = MOTOR_TICKS * GEAR_RATIO / (2 * PI)

    const val MIN_ANGLE = -135.0          // TODO: Verify - degrees
    const val MAX_ANGLE = 135.0           // TODO: Verify - degrees

    // ============================================
    // PID VALUES - TUNE THESE
    // ============================================
    var controller = controlSystem {
        posPid(0.3, 0.0, 0.05)   // kP, kI, kD
        basicFF(0.25, 0.0, 0.0)   // kV, kA, kS
    }

    @JvmField var minPower = 0.15         // Minimum power to overcome friction
    @JvmField var maxPower = 0.75         // Maximum safe power

    // ============================================
    // STATE
    // ============================================
    var currentState = State.IDLE
    var manualPower = 0.0

    // Velocity tracking
    private val velTimer = ElapsedTime()
    private var lastHeading = 0.0
    var angularVelocity = 0.0
        private set

    // Target
    private var targetAngle = 0.0  // radians
    private var targetVelocity = 0.0

    // Goal position (for aiming)
    @JvmField var goalX = 144.0
    @JvmField var goalY = 144.0

    // ============================================
    // INITIALIZATION
    // ============================================
    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        velTimer.reset()
    }

    // ============================================
    // MAIN LOOP
    // ============================================
    override fun periodic() {
        updateVelocity()

        when (currentState) {
            State.IDLE -> motor.power = 0.0
            State.MANUAL -> motor.power = manualPower.coerceIn(-maxPower, maxPower)
            State.AIMING -> applyControl()
        }
    }

    private fun applyControl() {
        controller.goal = KineticState(
            angleToTicks(targetAngle),
            targetVelocity
        )

        var power = controller.calculate(motor.state)

        // Minimum power threshold
        val error = targetAngle - getHeading()
        if (abs(Math.toDegrees(error)) > 0.5) {
            power += if (power >= 0) minPower else -minPower
        } else if (abs(targetVelocity) < 0.1) {
            power = 0.0
        }

        motor.power = power.coerceIn(-maxPower, maxPower)
    }

    private fun updateVelocity() {
        val dt = velTimer.seconds()
        if (dt > 0.001) {
            val currentHeading = getHeading()
            val delta = normalizeAngle(currentHeading - lastHeading)
            angularVelocity = delta / dt
            lastHeading = currentHeading
            velTimer.reset()
        }
    }

    // ============================================
    // PUBLIC API
    // ============================================

    /**
     * Set target angle for turret to aim at.
     * @param angleRad Target angle in radians
     * @param velocityComp Enable velocity compensation
     */
    fun setTarget(angleRad: Double, velocityComp: Boolean = true) {
        val minRad = degToRad(MIN_ANGLE)
        val maxRad = degToRad(MAX_ANGLE)
        targetAngle = angleRad.coerceIn(minRad, maxRad)
        targetVelocity = if (velocityComp) -angularVelocity * 0.25 else 0.0
    }

    /**
     * Set turret to manual control mode.
     */
    fun setManual(power: Double) {
        manualPower = power
        currentState = State.MANUAL
    }

    /**
     * Stop turret.
     */
    fun stop() {
        currentState = State.IDLE
        motor.power = 0.0
    }

    /**
     * Set goal position for odometry aiming.
     */
    fun setGoalPosition(x: Double, y: Double) {
        goalX = x
        goalY = y
    }

    /**
     * Get current turret heading in radians.
     */
    fun getHeading(): Double = motor.currentPosition.toDouble() / TICKS_PER_RADIAN

    /**
     * Normalize angle to [-PI, PI].
     */
    fun normalizeAngle(angle: Double): Double {
        var a = angle % (2 * PI)
        if (a <= -PI) a += 2 * PI
        if (a > PI) a -= 2 * PI
        return a
    }

    // Utilities
    private fun angleToTicks(rad: Double) = rad * TICKS_PER_RADIAN
    private fun ticksToAngle(ticks: Double) = ticks / TICKS_PER_RADIAN
    private fun degToRad(deg: Double) = deg * PI / 180.0
    private fun radToDeg(rad: Double) = rad * 180.0 / PI
}


/**
 * Odometry-based turret aim command.
 * Uses robot position and heading to calculate angle to goal.
 */
class OdometryAim(
    private val goalX: Double,        // Field X coordinate of goal
    private val goalY: Double,        // Field Y coordinate of goal
    private val poseX: () -> Double,  // Robot X supplier (live)
    private val poseY: () -> Double, // Robot Y supplier (live)
    private val poseH: () -> Double, // Robot heading supplier (RADIANS, normalized to [-PI, PI])
    private val ofsTurret: Angle = 0.0.rad  // Optional turret offset
) : Command() {

    // Continuous tracking - never completes
    override val isDone = false

    override fun start() {
        Turret.currentState = Turret.State.AIMING
    }

    override fun update() {
        // Get current robot pose
        val x = poseX()
        val y = poseY()
        val h = poseH()

        // Calculate angle from robot to goal
        val deltaX = goalX - x
        val deltaY = goalY - y
        val fieldAngle = atan2(deltaY, deltaX)  // Angle to goal in field space

        // Convert to robot-relative angle
        // turretAngle = where goal is relative to robot heading
        val turretAngle = fieldAngle - h + ofsTurret.inRad

        // Normalize to [-PI, PI] to minimize rotation
        val targetAngle = Turret.normalizeAngle(turretAngle)

        // Apply to turret with velocity compensation
        Turret.setTarget(targetAngle, compensateVelocity = true)
    }

    override fun stop(interrupted: Boolean) {
        if (!interrupted) {
            Turret.currentState = Turret.State.IDLE
        }
    }
}
