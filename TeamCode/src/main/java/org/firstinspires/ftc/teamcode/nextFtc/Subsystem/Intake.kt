package org.firstinspires.ftc.teamcode.nextFtc.Subsystem

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower

/**
 * Intake subsystem - simplified direct motor control
 * No command classes needed - just direct power control
 */
@Configurable
object Intake : Subsystem {

    private val motor = MotorEx("intake").reversed()

    // Direct power methods - call these from TeleOp
    var power: Double = 0.0
        private set

    /**
     * Set intake motor power directly
     * @param p Power from -1.0 (reverse) to 1.0 (forward/intake)
     */
    fun setPower(p: Double) {
        power = p.coerceIn(-1.0, 1.0)
        motor.power = power
    }

    /**
     * Run intake (forward) at full power
     */
    fun intake() {
        setPower(1.0)
    }

    /**
     * Run intake in reverse (out)
     */
    fun reverse() {
        setPower(-1.0)
    }

    /**
     * Stop intake
     */
    fun stop() {
        setPower(0.0)
    }

    override fun periodic() {
        // Nothing to update automatically - controlled directly by TeleOp
    }
}
