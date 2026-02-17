package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter

import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

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
    
    private val CLOSE_THRESHOLD = 20.0  // TODO: TUNE
    private val MID_THRESHOLD = 40.0    // TODO: TUNE

    // Use ServoEx with scale parameter - check your robot's direction!
    private val servo = ServoEx("hood", ServoEx.MAX_POSITION.toDouble())

    // Current target position for telemetry
    var currentTargetPosition: Double = MID
        private set

    // Robot position suppliers - set these from TeleOp
    var robotX: () -> Double = { 0.0 }
    var robotY: () -> Double = { 0.0 }
    var goalX: Double = 55.0  // Default goal position
    var goalY: Double = 0.0

    // Commands for common positions
    val full = SetPosition(servo, FAR)
    val close = SetPosition(servo, CLOSE)
    val half = SetPosition(servo, MID)
    val open = SetPosition(servo, DOWN)

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
     * Set hood position directly
     */
    fun setPosition(newPosition: Double) {
        currentTargetPosition = newPosition.coerceIn(0.0, 1.0)
        servo.position = currentTargetPosition
    }
    
    /**
     * Set hood position based on distance to target
     * Returns the position used
     */
    fun setForDistance(distanceInches: Double): Double {
        val position = when {
            distanceInches < CLOSE_THRESHOLD -> DOWN
            distanceInches < MID_THRESHOLD -> MID
            else -> FAR
        }
        setPosition(position)
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
        return kotlin.math.hypot(dx, dy)
    }
    
    override fun periodic() {
        // Auto-update hood position based on distance to goal!
        val distance = getDistanceToGoal()
        setForDistance(distance)

        // Debug telemetry
        PanelsTelemetry.telemetry.addData("Hood Distance", "%.1f".format(distance))
        PanelsTelemetry.telemetry.addData("Hood Position", "%.3f".format(currentPosition))
        PanelsTelemetry.telemetry.addData("Hood Target", "%.3f".format(currentTargetPosition))
    }
}
