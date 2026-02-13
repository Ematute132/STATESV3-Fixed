package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

/**
 * Hood subsystem for adjusting shooter angle based on distance.
 * 
 * Hardware: ServoEx connected to hood/angle adjustment mechanism
 * 
 * Position values (TUNE THESE for your robot!):
 * - DOWN/CLOSE = for close range shots
 * - MID = for medium range shots
 * - FAR = for long range shots
 */
object Hood : Subsystem {

    // TODO: TUNE THESE VALUES for your specific mechanism!
    @JvmField var DOWN = 0.0   // Close range
    @JvmField var CLOSE = 0.0  // Alias for DOWN
    @JvmField var MID = 0.5     // Medium range - TUNE THIS!
    @JvmField var FAR = 1.0     // Long range - TUNE THIS!

    private val servo = ServoEx("hood")

    val mid = SetPosition(servo, MID)
    val down = SetPosition(servo, DOWN)
    val closePos = SetPosition(servo, CLOSE)  // 'close' conflicts with Gate.close
    val far = SetPosition(servo, FAR)
    
    /**
     * Set hood position based on distance to target
     * Returns the position used (useful for telemetry/debugging)
     */
    fun setForDistance(distanceInches: Double): Double {
        val position = when {
            distanceInches < 20.0 -> DOWN
            distanceInches < 40.0 -> MID
            else -> FAR
        }
        servo.position = position
        return position
    }
    
    /**
     * Get current hood position
     */
    val currentPosition: Double get() = servo.position
    
    override fun periodic() {
        // Add any continuous updates needed
    }
}
