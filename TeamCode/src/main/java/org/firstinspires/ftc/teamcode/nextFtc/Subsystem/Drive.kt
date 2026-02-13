package org.firstinspires.ftc.teamcode.nextFtc.Subsystem

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower

/**
 * Drive subsystem using Pedro Pathing for odometry-based control.
 * 
 * IMPORTANT: You MUST call follower.update() in periodic() for pose to update!
 */
object Drive : Subsystem {
    
    // Pose accessors - use these in your code
    val poseX: Double get() = follower.pose.x
    val poseY: Double get() = follower.pose.y
    val poseHeading: Double get() = follower.pose.heading
    val poseValid: Boolean get() = follower.pose != null
    
    // Velocity accessors
    val velocityX: Double get() = follower.velocity.xComponent
    val velocityY: Double get() = follower.velocity.yComponent
    val velocityTheta: Double get() = follower.velocity.theta
    
    /**
     * CRITICAL: Call this in your TeleOp/Autonomous periodic() loop!
     * Without this, pose never updates and all odometry-based aim will fail.
     */
    fun updatePose() {
        follower.update()
    }
    
    /**
     * Get distance from current pose to a target point
     */
    fun distanceTo(targetX: Double, targetY: Double): Double {
        val dx = targetX - poseX
        val dy = targetY - poseY
        return kotlin.math.hypot(dx, dy)
    }
    
    /**
     * Reset pose to a specific position (useful for autonomous starting points)
     */
    fun resetPose(x: Double, y: Double, heading: Double) {
        follower.pose = com.pedropathing.geometry.Pose(x, y, heading)
    }
    
    override fun periodic() {
        // CRITICAL: Update Pedro follower every loop cycle
        // This is the #1 most common mistake - without this, pose never updates!
        follower.update()
    }
}
