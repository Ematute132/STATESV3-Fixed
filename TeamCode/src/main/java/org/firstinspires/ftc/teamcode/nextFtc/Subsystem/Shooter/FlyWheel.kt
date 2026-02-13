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
    
    // Control coefficients - TUNE THESE FOR YOUR ROBOT!
    // ============================================
    // HOW TO TUNE:
    // 1. Set all to 0
    // 2. Increase kV until motor reaches target velocity smoothly
    // 3. Add small kP to correct steady-state error
    // 4. Add kD to reduce overshoot
    // 5. kA usually not needed for flywheel
    // ============================================
    
    @JvmField var ffCoefficients = BasicFeedforwardParameters(
        kV = 0.003,  // TODO: TUNE - Start at 0.001, increase until reaches target velocity
        kA = 0.08,   // TODO: TUNE - For acceleration compensation (usually small)
        kS = 0.0     // TODO: TUNE - For friction (if motor drifts at low power)
    )
    
    @JvmField var pidCoefficients = PIDCoefficients(
        kP = 0.009,  // TODO: TUNE - Start at 0.005, increase for faster response
        kI = 0.0,    // Usually not needed for flywheel
        kD = 0.01    // TODO: TUNE - Helps reduce overshoot
    )
    
    // ============================================
    // VELOCITY PRESETS - TUNE THESE ON FIELD!
    // ============================================
    // Test at different distances and record velocity for each:
    // - Close: ~12-18 inches
    // - Mid: ~24-36 inches  
    // - Far: ~48+ inches
    // ============================================
    
    fun runClose() = setVelocity(1000.0)   // TODO: TUNE - Record actual velocity for close range
    fun runMid() = setVelocity(1250.0)    // TODO: TUNE - Record actual velocity for mid range
    fun runFar() = setVelocity(1500.0)    // TODO: TUNE - Record actual velocity for far range
    fun runMax() = setVelocity(2000.0)    // TODO: TUNE - Maximum safe velocity
    
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
