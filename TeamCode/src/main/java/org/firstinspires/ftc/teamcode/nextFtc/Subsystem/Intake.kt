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
     * Get current motor current for game piece detection
     * TODO: CHECK CORRECT NEXTFTC API FOR MOTOR CURRENT
     * Options: motor.currentDraw, motor.currentAlert, or remove this
     */
    // val current: Double get() = motor.current
    
    /**
     * Check if intake has a game piece
     * TODO: IMPLEMENT THIS WITH CORRECT MOTOR CURRENT API
     * Can use: motor.currentDraw, optical sensor, or time-based
     */
    fun hasGamePiece(): Boolean {
        // TODO: Replace with actual detection method
        return false
    }
    
    override fun periodic() {
        // Update state
        isRunning = kotlin.math.abs(motor.power) > 0.1
    }
}
