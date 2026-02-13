package org.firstinspires.ftc.teamcode.nextFtc.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.Turret
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech.OdometryAim
import kotlin.math.atan2
import kotlin.math.hypot

/**
 * Turret Odometry Aiming Test
 * Tests turret aiming using Pedro pathing odometry for position
 * 
 * Controls:
 * - Gamepad1 Left Stick: Move robot (Pedro driver control)
 * - Gamepad2 Right Stick: Manual turret control
 * - Gamepad2 Circle: Toggle odometry aiming at target
 * - Gamepad2 D-Pad: Cycle through target positions
 */
@Configurable
@TeleOp(name = "Turret Odometry Test", group = "Turret Tests")
class TurretOdoTest : NextFTCOpMode() {

    // Target positions to cycle through
    private val targets = listOf(
        Pair(72.0, 0.0),    // Speaker right
        Pair(72.0, 72.0),   // Speaker corner
        Pair(0.0, 72.0),    // Back wall center
        Pair(-72.0, 72.0),  // Amp corner
        Pair(-72.0, 0.0)    // Amp
    )
    private var currentTargetIndex = 0

    private var aimCommand: OdometryAim? = null
    private var isAiming = false

    init {
        addComponents(
            SubsystemComponent(Turret),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        // Manual turret control with right stick
        Gamepads.gamepad2.rightStickX.whenNotZero { value ->
            Turret.Manual { value }()
        }

        // Cycle targets with D-Pad
        Gamepads.gamepad2.dpadUp whenBecomesTrue {
            currentTargetIndex = (currentTargetIndex + 1) % targets.size
        }

        // Toggle odometry aiming
        Gamepads.gamepad2.circle whenBecomesTrue {
            if (isAiming) {
                Turret.stop()
                aimCommand = null
                isAiming = false
            } else {
                val target = targets[currentTargetIndex]
                aimCommand = OdometryAim(
                    goalX = target.first,
                    goalY = target.second,
                    x = { 0.0 },  // Will be updated in onUpdate
                    y = { 0.0 },  // Will be updated in onUpdate
                    h = { 0.0 }   // Will be updated in onUpdate
                )
                aimCommand!!()
                isAiming = true
            }
        }
    }

    override fun onUpdate() {
        CommandManager.cancelAll()

        // Update aim command with current pose if aiming
        if (isAiming && aimCommand != null) {
            val target = targets[currentTargetIndex]
            aimCommand = OdometryAim(
                goalX = target.first,
                goalY = target.second,
                x = { PedroComponent.follower.pose.x },
                y = { PedroComponent.follower.pose.y },
                h = { PedroComponent.follower.pose.heading }
            )
            aimCommand!!()
        }

        // Telemetry
        val pose = PedroComponent.follower.pose
        val target = targets[currentTargetIndex]
        val dx = target.first - pose.x
        val dy = target.second - pose.y
        val distance = hypot(dx, dy)
        val angleToTarget = Math.toDegrees(atan2(dy, dx))

        PanelsTelemetry.telemetry.addLine("=== TURRET ODOMETRY TEST ===")
        PanelsTelemetry.telemetry.addData("Robot X", pose.x)
        PanelsTelemetry.telemetry.addData("Robot Y", pose.y)
        PanelsTelemetry.telemetry.addData("Robot Heading", Math.toDegrees(pose.heading))
        PanelsTelemetry.telemetry.addData("Target X", target.first)
        PanelsTelemetry.telemetry.addData("Target Y", target.second)
        PanelsTelemetry.telemetry.addData("Distance to Target", distance)
        PanelsTelemetry.telemetry.addData("Angle to Target", angleToTarget)
        PanelsTelemetry.telemetry.addData("Turret Encoder", Turret.motor.currentPosition)
        PanelsTelemetry.telemetry.addData("Turret Angle (deg)", Math.toDegrees(Turret.turretYaw.inRad))
        PanelsTelemetry.telemetry.addData("Is Aiming", isAiming)
        PanelsTelemetry.telemetry.addData("Current Target Index", currentTargetIndex)
        PanelsTelemetry.telemetry.addData("Controller Goal", Turret.controller.goal.position)
        PanelsTelemetry.telemetry.addData("Controller Output", Turret.controller.calculate(Turret.motor.state))
        PanelsTelemetry.telemetry.update()
    }
}
