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

    // ============================================
    // HOOD POSITIONS - TUNE THESE ON FIELD!
    // ============================================
    // HOW TO TUNE:
    // 1. Set all to 0.5 (midpoint)
    // 2. Test at close range (~12 inches), adjust DOWN until shots go in
    // 3. Test at far range (~48 inches), adjust FAR until shots go in
    // 4. Test at mid range, adjust MID if needed
    // 5. Fine-tune based on trajectory
    // ============================================
    
    @JvmField var DOWN = 0.0   // TODO: TUNE - Close range (12-18 inches)
    @JvmField var CLOSE = 0.0  // Alias for DOWN
    @JvmField var MID = 0.5     // TODO: TUNE - Medium range (24-36 inches)
    @JvmField var FAR = 1.0     // TODO: TUNE - Long range (48+ inches)

    // ============================================
    // DISTANCE THRESHOLDS - ADJUST FOR YOUR FIELD!
    // ============================================
    // Change these based on where your robot typically positions
    // ============================================
    
    private val CLOSE_THRESHOLD = 20.0  // TODO: TUNE - Max distance for close hood
    private val MID_THRESHOLD = 40.0    // TODO: TUNE - Max distance for mid hood

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
            distanceInches < CLOSE_THRESHOLD -> DOWN
            distanceInches < MID_THRESHOLD -> MID
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
