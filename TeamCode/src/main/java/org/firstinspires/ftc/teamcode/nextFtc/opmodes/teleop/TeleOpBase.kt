package org.firstinspires.ftc.teamcode.nextFtc.opmodes.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.Data.Lefile
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.LL.LLBase
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Gate
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Intake
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.Hood
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.Turret
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech.TripleFusionAim
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

import java.io.File
import kotlin.math.hypot


data class ResetModeParams(val x: Double, val y: Double, val h: Angle)

@Configurable
open class TeleOpBase(
    private val isBlue: Boolean,
    private val goalX: Double,
    private val goalY: Double,
    private val resetModeParams: ResetModeParams,
    private val resetModePhiAngle: Angle,
    private val distanceToVelocity: (Double) -> Double,
    private val distanceToTheta: (Angle) -> Angle,
    private val distanceToTime: (Double) -> Double
): NextFTCOpMode() {

    // Pose accessors
    val x:  Double get() = PedroComponent.follower.pose.x
    val y:  Double get() = PedroComponent.follower.pose.y
    val h:  Angle  get() = PedroComponent.follower.pose.heading.rad
    val vx: Double get() = PedroComponent.follower.velocity.xComponent
    val vy: Double get() = PedroComponent.follower.velocity.yComponent
    val vh: Angle  get() = PedroComponent.follower.velocity.theta.rad

    var driverControlled: PedroDriverControlled? = null
    
    // Current aim command (for cancellation)
    private var aimCommand: TripleFusionAim? = null

    init {
        addComponents(
            SubsystemComponent(
                Turret,
                LLBase,
                Gate,
                Intake,
                Hood
            ),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        // Set up hood position providers
        Hood.setPositionProviders({ x }, { y })
        Hood.setGoalPosition(goalX, goalY)
        
        // Set turret goal position
        Turret.setGoalPosition(goalX, goalY)

        // Initialize Pedro Driver Controlled
        driverControlled = PedroDriverControlled(
            Gamepads.gamepad1.leftStickY.map { if (isBlue) it else -it },
            Gamepads.gamepad1.leftStickX.map { if (isBlue) it else -it },
            -Gamepads.gamepad1.rightStickX,
            false
        )

        // Load saved pose from file
        val file = File(Lefile.filePath)
        if (file.exists()) {
            try {
                val content = file.readText().split("\n")
                val startX = content.getOrNull(0)?.toDoubleOrNull() ?: 0.0
                val startY = content.getOrNull(1)?.toDoubleOrNull() ?: 0.0
                val startH = content.getOrNull(2)?.toDoubleOrNull() ?: 0.0
                PedroComponent.follower.pose = Pose(startX, startY, startH)
            } catch (e: Exception) {
                // If file parse fails, start at default position
            }
        }
    }

    private var autoAimEnabled = true
    private var resetMode = false
    private var phiTrim = 0.0.deg

    override fun onStartButtonPressed() {
        // Controller LEDs
        gamepad1.setLedColor(0.0, 0.0, 255.0, -1)
        gamepad2.setLedColor(255.0, 0.0, 0.0, -1)

        // Start Pedro driver control
        driverControlled?.let {
            it()
        }

        // ============================================
        // INTAKE CONTROL - Simple direct control
        // ============================================
        
        // Right trigger = intake, Left trigger = reverse
        Gamepads.gamepad1.rightTrigger.onChange { triggerValue ->
            if (triggerValue > 0.1) {
                Intake.setPower(triggerValue)
            } else if (Gamepads.gamepad1.leftTrigger.get() < 0.1) {
                Intake.stop()
            }
        }
        
        Gamepads.gamepad1.leftTrigger.onChange { triggerValue ->
            if (triggerValue > 0.1) {
                Intake.setPower(-triggerValue)
            } else if (Gamepads.gamepad1.rightTrigger.get() < 0.1) {
                Intake.stop()
            }
        }

        // Shooting (gate + intake) on right bumper
        Gamepads.gamepad1.rightBumper whenBecomesTrue {
            ParallelGroup(
                Gate.open,
                Intake.intake
            )()
        } whenBecomesFalse {
            ParallelGroup(
                Gate.close,
                Intake.stop
            )()
        }

        // ============================================
        // TURRET CONTROL - Kalman Fusion with MT1/MT2
        // ============================================

        // Hold circle to enable Kalman fusion aiming
        Gamepads.gamepad2.circle whenBecomesTrue {
            // Create and start the triple fusion aim command
            aimCommand = TripleFusionAim(
                goalX = goalX,
                goalY = goalY,
                poseX = { PedroComponent.follower.pose.x },
                poseY = { PedroComponent.follower.pose.y },
                poseHeading = { PedroComponent.follower.pose.heading }
            )
            aimCommand?.let {
                Turret.registerCommand(it)
                CommandManager.scheduleCommand(it)
            }
            gamepad2.rumble(100)
        } whenBecomesFalse {
            // Stop aiming when released
            Turret.stop()
            aimCommand = null
        }

        // Hold triangle for odometry-only aim (backup)
        Gamepads.gamepad2.triangle whenBecomesTrue {
            Turret.aimWithOdometry()
            gamepad2.rumble(50)
        } whenBecomesFalse {
            Turret.stop()
        }

        // Manual turret control with square
        Gamepads.gamepad2.square whenBecomesTrue {
            Turret.Manual(Gamepads.gamepad2.rightStickX.get() * 0.5)
        } whenBecomesFalse {
            Turret.stop()
        }

        // ============================================
        // RESET MODE
        // ============================================

        Gamepads.gamepad2.leftBumper and Gamepads.gamepad2.rightBumper whenBecomesTrue {
            resetMode = !resetMode
            if (resetMode) {
                gamepad2.rumble(200)
                gamepad2.setLedColor(255.0, 255.0, 0.0, -1)
            } else {
                // Reset position
                PedroComponent.follower.pose = Pose(
                    resetModeParams.x,
                    resetModeParams.y,
                    resetModeParams.h.inRad
                )
                gamepad2.rumble(200)
                gamepad2.setLedColor(255.0, 0.0, 0.0, -1)
            }
        }

        // Phi trim adjustment
        Gamepads.gamepad2.dpadRight whenBecomesTrue {
            phiTrim -= 2.0.deg
        }
        Gamepads.gamepad2.dpadLeft whenBecomesTrue {
            phiTrim += 2.0.deg
        }
    }

    var lastRuntime = 0.0

    override fun onUpdate() {
        val loopTime = runtime - lastRuntime
        lastRuntime = runtime

        // Update Pedro path follower
        PedroComponent.follower.update()

        // Distance calculations
        val dx = goalX - x
        val dy = goalY - y
        val dxy = hypot(dx, dy)
        val dxp = dx + vx * distanceToTime(dxy)
        val dyp = dy + vy * distanceToTime(dxy)
        val dxyp = hypot(dxp, dyp)

        // Standard telemetry
        telemetry.addData("Loop Time (ms)", "%.1f".format(loopTime))
        telemetry.addData("x (inch)", "%.1f".format(x))
        telemetry.addData("y (inch)", "%.1f".format(y))
        telemetry.addData("h (deg)", "%.1f".format(Math.toDegrees(h.inRad)))
        telemetry.addData("distanceToGoal", "%.1f".format(dxy))
        telemetry.addData("Reset Mode", resetMode)
        
        // Intake telemetry
        telemetry.addData("Intake Power", "%.2f".format(Intake.power))
        
        telemetry.update()

        // Panels telemetry
        PanelsTelemetry.telemetry.addData("Loop Time", "%.1fms".format(loopTime))
        PanelsTelemetry.telemetry.addData("Pose", "(%.1f, %.1f, %.1f)".format(x, y, Math.toDegrees(h.inRad)))
        PanelsTelemetry.telemetry.addData("Distance", "%.1f".format(dxy))
        PanelsTelemetry.telemetry.addData("CMD", CommandManager.snapshot)
        PanelsTelemetry.telemetry.update()
    }

    override fun onStop() {
        // Save pose to file
        try {
            val file = File(Lefile.filePath)
            file.writeText(
                "%.6f\n%.6f\n%.6f".format(x, y, h.inRad)
            )
        } catch (e: Exception) {
            // Ignore
        }
        
        // Stop all subsystems
        Intake.stop()
        Turret.stop()
    }
}
