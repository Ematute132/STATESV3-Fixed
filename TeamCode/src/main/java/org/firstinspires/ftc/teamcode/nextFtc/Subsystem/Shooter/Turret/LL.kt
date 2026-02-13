package org.firstinspires.ftc.teamcode.Subsystem.Shooter

import dev.nextftc.core.commands.Command
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Vision.LLBase
import java.util.function.Supplier
import kotlin.math.atan2


class LimelightAim(
    private val ofsTurret: Angle = 0.0.rad
) : Command() {

    override val isDone = false

    override fun start() {
        Turret.currentState = Turret.State.VISION_AIM
        Turret.registerCommand(this)
    }

    override fun update() {
        val result = LLBase.ll.latestResult

        // Only aim if we have a valid target
        if (result == null || !result.isValid) {
            // No target - hold current position
            return
        }

        // tx is the horizontal offset to target in degrees
        // Positive tx = target is to the right = turret needs to turn right
        val tx = result.tx

        // Current turret angle + tx offset = where we need to aim
        val currentAngle = Turret.turretYaw
        val targetAngle = Turret.normalizeAngle(currentAngle + tx.deg + ofsTurret)

        Turret.setTargetAngle(targetAngle, compensateVelocity = true)
    }

    override fun stop(interrupted: Boolean) {
        if (!interrupted) Turret.currentState = Turret.State.IDLE
    }
}

/**
 * Vision-based aiming using custom dx/dy suppliers.
 *
 * For when you want to compute target position yourself
 * (e.g., from AprilTag pose estimation).
 */
class LimelightAimCustom(
    private val dx: Supplier<Double>,
    private val dy: Supplier<Double>,
    private val robotHeading: Supplier<Angle>,
    private val ofsTurret: Angle = 0.0.rad
) : Command() {

    override val isDone = false

    override fun start() {
        Turret.currentState = Turret.State.VISION_AIM
        Turret.registerCommand(this)
    }

    override fun update() {
        val targetAngle = Turret.normalizeAngle(
            atan2(dy.get(), dx.get()).rad - robotHeading.get() + ofsTurret
        )
        Turret.setTargetAngle(targetAngle, compensateVelocity = true)
    }

    override fun stop(interrupted: Boolean) {
        if (!interrupted) Turret.currentState = Turret.State.IDLE
    }
}

/**
 * Vision-based aiming using Limelight MegaTag2 botpose.
 *
 * Uses the MT2 field-relative position to compute angle to goal.
 * More accurate than tx when you have good AprilTag visibility.
 */
class LimelightAimMT2(
    private val goalX: Double,
    private val goalY: Double,
    private val ofsTurret: Angle = 0.0.rad
) : Command() {

    override val isDone = false

    override fun start() {
        Turret.currentState = Turret.State.VISION_AIM
        Turret.registerCommand(this)
    }

    override fun update() {
        val result = LLBase.ll.latestResult
        if (result == null || !result.isValid) return

        val mt2 = result.botpose_MT2 ?: return

        val robotX = mt2.position.x
        val robotY = mt2.position.y
        val robotYaw = mt2.orientation.yaw  // In degrees from LL

        val deltaX = goalX - robotX
        val deltaY = goalY - robotY
        val fieldAngle = atan2(deltaY, deltaX)

        val robotHeadingRad = Math.toRadians(robotYaw)

        val targetAngle = Turret.normalizeAngle(
            (fieldAngle - robotHeadingRad).rad + ofsTurret
        )

        Turret.setTargetAngle(targetAngle, compensateVelocity = true)
    }

    override fun stop(interrupted: Boolean) {
        if (!interrupted) Turret.currentState = Turret.State.IDLE
    }
}


