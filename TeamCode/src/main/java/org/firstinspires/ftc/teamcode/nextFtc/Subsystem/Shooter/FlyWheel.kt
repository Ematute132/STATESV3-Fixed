package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry.telemetry
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import java.util.function.Supplier
import kotlin.math.abs

/**
 * Flywheel shooter subsystem with PID + Feedforward control.
 * 
 * Hardware: 2 MotorEx motors (Fly1, Fly2) in push-pull configuration
 * 
 * Features:
 * - Velocity-based control using KineticState
 * - PID feedback for accuracy
 * - Feedforward for smooth acceleration
 * - Preset velocities for different distances
 */
@Configurable
object FlyWheel : Subsystem {
    
    // Hardware definition
    private val motor1 = MotorEx("Fly1")
    private val motor2 = MotorEx("Fly2").reversed()
    
    // Control coefficients - TUNE THESE for your robot!
    @JvmField var ffCoefficients = BasicFeedforwardParameters(0.003, 0.08, 0.0)  // kV, kA, kS
    @JvmField var pidCoefficients = PIDCoefficients(0.009, 0.0, 0.01)  // kP, kI, kD
    
    // Control system
    val controller: ControlSystem = controlSystem {
        basicFF(ffCoefficients)
        velPid(pidCoefficients)
    }
    
    // Current state
    var isRunning: Boolean = false
        private set
    var targetVelocity: Double = 0.0
        private set
    
    /**
     * Set target velocity (RPM)
     * @param velocity Target velocity in RPM
     */
    fun setVelocity(velocity: Double) {
        targetVelocity = velocity
        // KineticState(position, velocity) - position 0 for pure velocity control
        controller.goal = KineticState(0.0, velocity)
        isRunning = velocity > 50.0
    }
    
    // Preset velocities - TUNE THESE for your robot!
    fun runClose() = setVelocity(1000.0)   // For close range
    fun runMid() = setVelocity(1250.0)    // For medium range  
    fun runFar() = setVelocity(1500.0)    // For long range
    fun runMax() = setVelocity(2000.0)    // Maximum
    
    /**
     * Manual power control (for testing/troubleshooting)
     */
    class Manual(private val power: Supplier<Double>) : Command() {
        override val isDone = false
        init { requires(this@FlyWheel) }
        
        override fun update() {
            motor1.power = power.get()
            motor2.power = power.get()
        }
    }
    
    /**
     * Stop the flywheel
     */
    val off = InstantCommand {
        setVelocity(0.0)
        motor1.power = 0.0
        motor2.power = 0.0
        isRunning = false
    }
    
    /**
     * Check if both motors are synchronized (within tolerance)
     */
    fun isSynced(tolerance: Double = 100.0): Boolean {
        return abs(motor1.velocity - motor2.velocity) < tolerance
    }
    
    /**
     * Get average velocity of both motors
     */
    val averageVelocity: Double get() = (motor1.velocity + motor2.velocity) / 2.0
    
    override fun periodic() {
        // Calculate control effort
        val controlEffort = controller.calculate(motor1.state)
        
        // Apply to motors (clamp for safety)
        val clampedPower = controlEffort.coerceIn(-0.85, 0.85)
        motor1.power = clampedPower
        motor2.power = clampedPower
        
        // Telemetry for debugging
        telemetry.addData("Flywheel Power", clampedPower)
        telemetry.addData("Target Vel", targetVelocity)
        telemetry.addData("Actual Vel", motor1.velocity)
        telemetry.addData("Motor 2 Vel", motor2.velocity)
        telemetry.addData("Synced", isSynced())
    }
}
