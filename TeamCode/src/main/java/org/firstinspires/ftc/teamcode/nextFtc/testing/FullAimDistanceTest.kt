package org.firstinspires.ftc.teamcode.nextFtc.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Gate
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Intake
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.LL.LLBase
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.Hood
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.Turret
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech.TripleFusionAim
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.roundToInt

/**
 * Full Aiming Distance Test
 * Tests complete shooting pipeline:
 * - Distance calculation to target
 * - Turret aiming (fusion)
 * - Flywheel velocity selection based on distance
 * - Hood position selection based on distance
 * - Combined shooting action
 * 
 * Controls:
 * - Gamepad1 Left Stick: Move robot
 * - Gamepad1 Right Bumper: Shoot (fires and cycles)
 * - Gamepad1 Left Trigger: Reverse intake
 * - Gamepad2 Circle: Toggle turret auto-aim
 * - Gamepad2 Right Stick: Manual turret
 * - Gamepad2 X/Y/B/A: Set flywheel preset (close/mid/far/stop)
 */
@Configurable
@TeleOp(name = "Full Aim Distance Test", group = "Integration Tests")
class FullAimDistanceTest : NextFTCOpMode() {

    // Goal position (adjust for your field)
    private val goalX = 72.0
    private val goalY = 0.0

    private var fusionCommand: TripleFusionAim? = null
    private var isAiming = false
    
    // Distance ranges (TUNE THESE!)
    private val closeMax = 24.0
    private val midMax = 48.0
    
    // Velocity presets
    private val closeVel = 1000.0
    private val midVel = 1250.0
    private val farVel = 1500.0
    
    // Hood positions
    private val closeHood = 0.0
    private val midHood = 0.5
    private val farHood = 1.0

    init {
        addComponents(
            SubsystemComponent(Turret),
            SubsystemComponent(FlyWheel),
            SubsystemComponent(Hood),
            SubsystemComponent(Gate),
            SubsystemComponent(Intake),
            SubsystemComponent(LLBase),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        // Toggle turret aiming
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

        // Manual turret
        Gamepads.gamepad2.rightStickX.whenNotZero { value ->
            Turret.Manual { value }()
        }

        // Flywheel presets
        Gamepads.gamepad2.x whenBecomesTrue {
            FlyWheel.runClose()
            Hood.down
        }
        Gamepads.gamepad2.y whenBecomesTrue {
            FlyWheel.runMid()
            Hood.mid
        }
        Gamepads.gamepad2.b whenBecomesTrue {
            FlyWheel.runFar()
            Hood.far
        }
        Gamepads.gamepad2.a whenBecomesTrue {
            FlyWheel.setVelocity(0.0)
        }

        // Shoot action
        Gamepads.gamepad1.rightBumper whenBecomesTrue {
            ParallelGroup(
                Gate.open,
                Intake.intake
            )()
        } whenBecomesFalse {
            ParallelGroup(
                Gate.close,
                Intake.off
            )()
        }

        // Intake reverse
        Gamepads.gamepad1.leftTrigger.whenNotZero { 
            Intake.reverse() 
        }
    }

    override fun onUpdate() {
        CommandManager.cancelAll()

        // Update fusion if active
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

        // Calculate distance to goal
        val pose = PedroComponent.follower.pose
        val dx = goalX - pose.x
        val dy = goalY - pose.y
        val distance = hypot(dx, dy)
        
        // Calculate angle to goal
        val angleToGoal = Math.toDegrees(atan2(dy, dx))
        
        // Determine range based on distance
        val range = when {
            distance <= closeMax -> "CLOSE"
            distance <= midMax -> "MID"
            else -> "FAR"
        }

        // Get selected values for current distance
        val selectedVelocity = when (range) {
            "CLOSE" -> closeVel
            "MID" -> midVel
            else -> farVel
        }
        
        val selectedHood = when (range) {
            "CLOSE" -> closeHood
            "MID" -> midHood
            else -> farHood
        }

        // Telemetry
        PanelsTelemetry.telemetry.addLine("=== FULL AIM DISTANCE TEST ===")
        
        // Robot pose
        PanelsTelemetry.telemetry.addData("Robot X", pose.x)
        PanelsTelemetry.telemetry.addData("Robot Y", pose.y)
        PanelsTelemetry.telemetry.addData("Robot Heading", Math.toDegrees(pose.heading))
        
        // Target info
        PanelsTelemetry.telemetry.addData("Goal X", goalX)
        PanelsTelemetry.telemetry.addData("Goal Y", goalY)
        PanelsTelemetry.telemetry.addData("Delta X", dx)
        PanelsTelemetry.telemetry.addData("Delta Y", dy)
        PanelsTelemetry.telemetry.addData("Distance", distance.roundToInt())
        PanelsTelemetry.telemetry.addData("Angle to Goal", angleToGoal.roundToInt())
        
        // Range selection
        PanelsTelemetry.telemetry.addData("Range", range)
        PanelsTelemetry.telemetry.addData("Close Max", closeMax)
        PanelsTelemetry.telemetry.addData("Mid Max", midMax)
        
        // Selected values
        PanelsTelemetry.telemetry.addData("Selected Vel", selectedVelocity)
        PanelsTelemetry.telemetry.addData("Selected Hood", selectedHood)
        PanelsTelemetry.telemetry.addData("Actual Vel", FlyWheel.controller.state.velocity)
        PanelsTelemetry.telemetry.addData("Actual Hood Pos", Hood.currentPosition)
        
        // Turret
        PanelsTelemetry.telemetry.addData("Turret Angle", Math.toDegrees(Turret.turretYaw.inRad).roundToInt())
        PanelsTelemetry.telemetry.addData("Is Aiming", isAiming)
        
        // Status
        PanelsTelemetry.telemetry.addData("Flywheel Running", FlyWheel.isRunning)
        PanelsTelemetry.telemetry.addData("Gate Open", Gate.isOpen)
        
        // Controls help
        PanelsTelemetry.telemetry.addLine("Controls:")
        PanelsTelemetry.telemetry.addData("G2 Circle: Toggle Aim", "")
        PanelsTelemetry.telemetry.addData("G2 X/Y/B/A: Presets", "")
        PanelsTelemetry.telemetry.addData("G1 RB: Shoot", "")
        
        PanelsTelemetry.telemetry.update()
    }
}
