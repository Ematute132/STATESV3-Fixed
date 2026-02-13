package org.firstinspires.ftc.teamcode.nextFtc.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.LL.LLBase
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.Turret
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech.TripleFusionAim
import kotlin.math.hypot

/**
 * Turret Fusion Aiming Test
 * Tests turret aiming using Triple-Source Kalman Fusion (Odo + MT2 + MT1)
 * This combines all three sources for the most accurate aiming
 * 
 * Controls:
 * - Gamepad2 Circle: Toggle fusion aiming at speaker
 * - Gamepad2 Right Stick: Manual turret control
 * - Gamepad2 D-Pad Up/Down: Increase/decrease fusion noise (for testing)
 */
@Configurable
@TeleOp(name = "Turret Fusion Test", group = "Turret Tests")
class TurretFusionTest : NextFTCOpMode() {

    private var fusionCommand: TripleFusionAim? = null
    private var isAiming = false

    // Goal position
    private val goalX = 72.0
    private val goalY = 0.0

    init {
        addComponents(
            SubsystemComponent(Turret),
            SubsystemComponent(LLBase),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        // Manual turret control
        Gamepads.gamepad2.rightStickX.whenNotZero { value ->
            Turret.Manual { value }()
        }

        // Toggle fusion aiming
        Gamepads.gamepad2.circle whenBecomesTrue {
            if (isAiming) {
                Turret.stop()
                fusionCommand = null
                isAiming = false
            } else {
                fusionCommand = TripleFusionAim(
                    goalX = goalX,
                    goalY = goalY,
                    poseX = { PedroComponent.follower.pose.x },
                    poseY = { PedroComponent.follower.pose.y },
                    poseHeading = { PedroComponent.follower.pose.heading }
                )
                fusionCommand!!()
                isAiming = true
            }
        }

        // Adjust noise parameters for testing
        Gamepads.gamepad2.dpadUp whenBecomesTrue {
            TripleFusionAim.processNoiseQ += 0.001
        }
        Gamepads.gamepad2.dpadDown whenBecomesTrue {
            TripleFusionAim.processNoiseQ = (TripleFusionAim.processNoiseQ - 0.001).coerceAtLeast(0.0001)
        }
    }

    override fun onUpdate() {
        CommandManager.cancelAll()

        // Update fusion command if active
        if (isAiming && fusionCommand != null) {
            fusionCommand = TripleFusionAim(
                goalX = goalX,
                goalY = goalY,
                poseX = { PedroComponent.follower.pose.x },
                poseY = { PedroComponent.follower.pose.y },
                poseHeading = { PedroComponent.follower.pose.heading }
            )
            fusionCommand!!()
        }

        // Get robot pose
        val pose = PedroComponent.follower.pose
        val distance = hypot(goalX - pose.x, goalY - pose.y)

        // Get Kalman filter state from fusion command
        val kfAngle = if (fusionCommand != null) {
            // This is a workaround - ideally we'd expose the KF state
            Turret.turretYaw.inRad
        } else {
            Turret.turretYaw.inRad
        }

        // Telemetry
        PanelsTelemetry.telemetry.addLine("=== TURRET FUSION TEST ===")
        PanelsTelemetry.telemetry.addData("Robot X", pose.x)
        PanelsTelemetry.telemetry.addData("Robot Y", pose.y)
        PanelsTelemetry.telemetry.addData("Robot Heading", Math.toDegrees(pose.heading))
        PanelsTelemetry.telemetry.addData("Distance to Goal", distance)
        PanelsTelemetry.telemetry.addData("Turret Angle (deg)", Math.toDegrees(Turret.turretYaw.inRad))
        PanelsTelemetry.telemetry.addData("Turret Encoder", Turret.motor.currentPosition)
        PanelsTelemetry.telemetry.addData("Is Aiming", isAiming)
        PanelsTelemetry.telemetry.addData("KF Process Noise Q", TripleFusionAim.processNoiseQ)
        PanelsTelemetry.telemetry.addData("KF MT2 Noise R", TripleFusionAim.mt2NoiseR)
        PanelsTelemetry.telemetry.addData("KF MT1 Noise R", TripleFusionAim.mt1NoiseR)
        PanelsTelemetry.telemetry.addData("Controller Goal", Turret.controller.goal.position)
        PanelsTelemetry.telemetry.addData("Controller Output", Turret.controller.calculate(Turret.motor.state))
        PanelsTelemetry.telemetry.addData("Robot Angular Vel", Math.toDegrees(Turret.robotAngularVelocity))
        PanelsTelemetry.telemetry.update()
    }
}
