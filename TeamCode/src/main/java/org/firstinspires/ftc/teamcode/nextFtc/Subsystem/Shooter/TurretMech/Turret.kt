package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2

/**
 * Base turret subsystem.
 * Works with OdometryAim, VisionAim, and Kalman fusion commands.
 */
object Turret : Subsystem {

    enum class State { IDLE, MANUAL, AIMING }

    var motor = MotorEx("turret")

    // ============================================
    // PHYSICAL CONSTANTS - MEASURE YOUR ROBOT
    // ============================================
    val GEAR_RATIO = 105.0 / 29.0
    const val MOTOR_TICKS = 537.7
    private val TICKS_PER_RADIAN = MOTOR_TICKS * GEAR_RATIO / (2 * PI)

    const val MIN_ANGLE_DEG = -135.0
    const val MAX_ANGLE_DEG = 135.0

    // ============================================
    // PID VALUES - TUNE THESE
    // ============================================
    var controller = controlSystem {
        posPid(0.3, 0.0, 0.05)
        basicFF(0.25, 0.0, 0.0)
    }

    @JvmField var minPower = 0.15
    @JvmField var maxPower = 0.75

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
    private var targetAngle = 0.0
    private var targetVelocity = 0.0

    // Goal position
    @JvmField var goalX = 144.0
    @JvmField var goalY = 144.0

    // Command tracking
    internal var lastCommand: Command? = null

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

        val error = normalizeAngle(targetAngle - getHeading())
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

    fun setTarget(angleRad: Double, velocityComp: Boolean = true) {
        val minRad = degToRad(MIN_ANGLE_DEG)
        val maxRad = degToRad(MAX_ANGLE_DEG)
        targetAngle = angleRad.coerceIn(minRad, maxRad)
        targetVelocity = if (velocityComp) -angularVelocity * 0.25 else 0.0
    }

    fun setManual(power: Double) {
        manualPower = power
        currentState = State.MANUAL
    }

    fun stop() {
        currentState = State.IDLE
        motor.power = 0.0
        lastCommand?.let {
            CommandManager.cancelCommand(it)
            lastCommand = null
        }
    }

    fun setGoalPosition(x: Double, y: Double) {
        goalX = x
        goalY = y
    }

    fun getHeading(): Double = normalizeAngle(motor.currentPosition.toDouble() / TICKS_PER_RADIAN)

    fun normalizeAngle(angle: Double): Double {
        var a = angle % (2 * PI)
        if (a <= -PI) a += 2 * PI
        if (a > PI) a -= 2 * PI
        return a
    }

    fun registerCommand(command: Command) {
        if (lastCommand != null && lastCommand != command) {
            CommandManager.cancelCommand(lastCommand!!)
        }
        lastCommand = command
    }

    private fun angleToTicks(rad: Double) = rad * TICKS_PER_RADIAN
    private fun degToRad(deg: Double) = deg * PI / 180.0
}


/**
 * Odometry-based turret aim command.
 */
class OdometryAim(
    private val goalX: Double,
    private val goalY: Double,
    private val poseX: () -> Double,
    private val poseY: () -> Double,
    private val poseH: () -> Double,
    private val ofsTurret: Angle = 0.0.rad
) : Command() {

    override val isDone = false

    override fun start() {
        Turret.currentState = Turret.State.AIMING
        Turret.registerCommand(this)
    }

    override fun update() {
        val x = poseX()
        val y = poseY()
        val h = poseH()

        val deltaX = goalX - x
        val deltaY = goalY - y
        val fieldAngle = atan2(deltaY, deltaX)

        val turretAngle = fieldAngle - h + ofsTurret.inRad
        val targetAngle = Turret.normalizeAngle(turretAngle)

        Turret.setTarget(targetAngle, compensateVelocity = true)
    }

    override fun stop(interrupted: Boolean) {
        if (!interrupted) {
            Turret.currentState = Turret.State.IDLE
        }
    }
}
