package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.Turret

import dev.nextftc.core.commands.Command
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2

class OdometryAim(
    private val goalX: Double,              // Field X coordinate of goal
    private val goalY: Double,
    private val x: Double ,
    private val y: Double,
    private val h: Double,// Field Y coordinate of goal
    private val ofsTurret: Angle = 0.0.rad  // Optional turret offset
) : Command() {

    override val isDone = false  // Continuous tracking

    override fun start() {
        Turret.currentState = Turret.State.ODOMETRY_AIM
        Turret.registerCommand(this)
    }

    override fun update() {
        val deltaX = goalX - x
        val deltaY = goalY - y
        val fieldAngle = atan2(deltaY, deltaX)

        // Handle heading that might be in degrees vs radians
        val robotHeading = h.let { heading ->
            if (abs(heading) > 2.0 * PI) Math.toRadians(heading) else heading
        }

        val targetAngle = Turret.normalizeAngle(
            (fieldAngle - robotHeading).rad + ofsTurret
        )

        Turret.setTargetAngle(targetAngle, compensateVelocity = true)
    }

    override fun stop(interrupted: Boolean) {
        if (!interrupted) Turret.currentState = Turret.State.IDLE
    }
}

/**
 * Odometry aiming using Turret's internal goal suppliers.
 *
 * Wire up Turret.goalX and Turret.goalY before using this command.
 * Useful for dynamic goal selection (e.g., alliance-aware targeting).
 *
 * Usage:
 * ```kotlin
 * // Set goal based on alliance
 * Turret.goalX = { if (isRed) 72.0 else -72.0 }
 * Turret.goalY = { 0.0 }
 *
 * CommandManager.scheduleCommand(OdometryAimInternal())
 * ```
 */

fun computeOdometryAngle(
    goalX: Double,
    goalY: Double,
    x: Double,
    y: Double,
    h: Double,
    ofsTurret: Angle = 0.0.rad
): Angle? {

    val deltaX = goalX - x
    val deltaY = goalY - y
    val fieldAngle = atan2(deltaY, deltaX)

    val robotHeading = h.let { heading ->
        if (abs(heading) > 2.0 * PI) Math.toRadians(heading) else heading
    }

    return Turret.normalizeAngle((fieldAngle - robotHeading).rad + ofsTurret)
}