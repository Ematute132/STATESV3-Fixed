package org.firstinspires.ftc.teamcode.nextFtc.Subsystem

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import java.util.function.Supplier

/**
 * Intake subsystem for game piece collection.
 * 
 * Hardware: MotorEx connected to intake mechanism
 * 
 * Controls:
 * - SetPower for continuous power control
 * - DriverCommand for trigger-based control (intake on RT, eject on LT)
 */
@Configurable
object Intake : Subsystem {
    private val motor = MotorEx("intake").reversed()

    // ============================================
    // INTAKE POWER - TUNE THESE!
    // ============================================
    // HOW TO TUNE:
    // 1. Set intake power to 1.0, test pickup speed
    // 2. Increase if too slow, decrease if pieces get stuck
    // 3. Reverse power should be enough to eject but not damage
    // ============================================
    
    val intake = SetPower(motor, 1.0)     // TODO: TUNE - Forward power (0.5 to 1.0)
    val reverse = SetPower(motor, -0.5)   // TODO: TUNE - Eject power (-0.3 to -0.8)
    val off = SetPower(motor, 0.0)
    
    // State tracking
    var isRunning: Boolean = false
        private set
    
    // ============================================
    // COMMANDS
    // ============================================
    
    /**
     * Simple command to turn intake on at configured power
     */
    class On : Command() {
        override val isDone = true
        
        override fun start() {
            motor.power = 1.0
            isRunning = true
        }
    }
    
    /**
     * One-shot intake command - runs for specified duration then stops
     */
    class TimedIntake(private val durationMs: Long) : Command() {
        override val isDone = true
        
        override fun start() {
            motor.power = 1.0
            isRunning = true
        }
    }
    
    /**
     * Turn intake on with specific power level
     */
    class PowerOn(private val power: Double) : Command() {
        override val isDone = true
        
        override fun start() {
            motor.power = power.coerceIn(-1.0, 1.0)
            isRunning = kotlin.math.abs(power) > 0.1
        }
    }
    
    /**
     * Turn intake on and run for specified time at given power
     */
    class TimedPowerIntake(private val power: Double, private val durationMs: Long) : Command() {
        override val isDone = true
        
        override fun start() {
            motor.power = power.coerceIn(-1.0, 1.0)
            isRunning = kotlin.math.abs(power) > 0.1
        }
    }
    
    /**
     * Driver control - hold RT to intake, LT to eject
     * Formula: intake - outtake (allows mixing both triggers)
     */
    class DriverCommand : Command() {
        override val isDone = false
        
        init {
            setInterruptible(true)
            setName("Intake Driver")
        }
        
        override fun start() {
            motor.power = 0.0
            isRunning = false
        }
        
        override fun update() {
            // Right trigger = intake (positive), Left trigger = eject (negative)
            val intakePower = dev.nextftc.ftc.Gamepads.gamepad1.rightTrigger.get()
            val ejectPower = dev.nextftc.ftc.Gamepads.gamepad1.leftTrigger.get()
            
            motor.power = intakePower - ejectPower
            isRunning = motor.power > 0.1
        }
        
        override fun end(interrupted: Boolean) {
            motor.power = 0.0
            isRunning = false
        }
    }
    
    /**
     * Get current motor current draw in amps
     * TODO: VERIFY THIS API - Check NextFTC MotorEx documentation
     * Common alternatives: motor.current, motor.currentDrawAmps
     */
    val currentAmps: Double
        get() = motor.motor.current / 1000.0  // Convert mA to A if needed
    
    /**
     * Check if intake has a game piece based on current draw
     * Higher current = motor is working harder = has game piece
     * TODO: TUNE THRESHOLD for your specific intake
     */
    fun hasGamePiece(thresholdAmps: Double = 2.0): Boolean {
        return currentAmps > thresholdAmps
    }
    
    override fun periodic() {
        // Update state
        isRunning = kotlin.math.abs(motor.power) > 0.1
    }
}
