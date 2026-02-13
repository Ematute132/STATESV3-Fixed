package org.firstinspires.ftc.teamcode.nextFtc.Subsystem

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

/**
 * Gate subsystem for controlling game piece release.
 * 
 * Hardware: ServoEx connected to gate mechanism
 * 
 * Position values (TUNE THESE for your robot!):
 * - OPEN = 0.0 (or whatever lets pieces through)
 * - CLOSE = 1.0 (or whatever blocks pieces)
 * 
 * HOW TO TUNE:
 * 1. Set OPEN = 0.5, CLOSE = 0.5
 * 2. Manually move gate to OPEN position, record value
 * 3. Manually move gate to CLOSE position, record value
 * 4. Update OPEN and CLOSE with measured values
 */
object Gate : Subsystem {
    
    // ============================================
    // GATE POSITIONS - TUNE THESE!
    // ============================================
    
    @JvmField var OPEN = 0.0   // TODO: MEASURE - Position when gate is open (pieces fall through)
    @JvmField var CLOSE = 1.0  // TODO: MEASURE - Position when gate is closed (blocks pieces)

    private val servo = ServoEx("gate")

    val close = SetPosition(servo, CLOSE)
    val open = SetPosition(servo, OPEN)
    
    // State tracking
    var isOpen: Boolean = false
        private set
    
    /**
     * Toggle between open and closed
     */
    val toggle = SetPosition(servo) {
        if (isOpen) {
            servo.position = CLOSE
            isOpen = false
        } else {
            servo.position = OPEN
            isOpen = true
        }
    }
    
    override fun periodic() {
        isOpen = kotlin.math.abs(servo.position - OPEN) < 0.1
    }
}
