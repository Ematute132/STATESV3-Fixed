package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter

import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import kotlin.math.hypot

/**
 * Hood subsystem for adjusting shooter angle based on distance.
 * NOW auto-updates in periodic() based on robot position!
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

    // Current target position for telemetry
    var currentTargetPosition: Double = MID
        private set

    // Robot position suppliers - set these from TeleOp
    var robotX: () -> Double = { 0.0 }
    var robotY: () -> Double = { 0.0 }
    var goalX: Double = 55.0  // Default goal position
    var goalY: Double = 0.0

    val mid = SetPosition(servo, MID)
    val down = SetPosition(servo, DOWN)
    val closePos = SetPosition(servo, CLOSE)  // 'close' conflicts with Gate.close
    val far = SetPosition(servo, FAR)
    
    /**
     * Set goal position for distance calculation
     */
    fun setGoalPosition(gx: Double, gy: Double) {
        goalX = gx
        goalY = gy
    }

    /**
     * Set position providers from TeleOp
     */
    fun setPositionProviders(x: () -> Double, y: () -> Double) {
        robotX = x
        robotY = y
    }
    
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
        currentTargetPosition = position
        return position
    }
    
    /**
     * Get current hood position
     */
    val currentPosition: Double get() = servo.position

    /**
     * Get distance to goal
     */
    private fun getDistanceToGoal(): Double {
        val dx = goalX - robotX()
        val dy = goalY - robotY()
        return hypot(dx, dy)
    }
    
    override fun periodic() {
        // Auto-update hood position based on distance to goal!
        val distance = getDistanceToGoal()
        setForDistance(distance)

        // Debug telemetry
        PanelsTelemetry.telemetry.addData("Hood Distance", distance)
        PanelsTelemetry.telemetry.addData("Hood Position", currentPosition)
        PanelsTelemetry.telemetry.addData("Hood Target", currentTargetPosition)
    }
}
