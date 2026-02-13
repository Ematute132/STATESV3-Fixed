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
    private val distanceToTheta: (Double) -> Angle,
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

    // Turret aiming command (stored so we can cancel it)
    private var aimCommand: TripleFusionAim? = null

    init {
        addComponents(
            SubsystemComponent(
                Turret,
                LLBase,
                Gate,
                Intake
            ),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        driverControlled = PedroDriverControlled(
            Gamepads.gamepad1.leftStickY.map { if (isBlue) it else -it },
            Gamepads.gamepad1.leftStickX.map { if (isBlue) it else -it },
            -Gamepads.gamepad1.rightStickX,
            false
        )

        // Load saved pose from file with error handling
        val file = File(Lefile.filePath)
        if (file.exists()) {
            try {
                val content = file.readText().split("\n")
                if (content.size >= 3) {
                    val startX = content[0].toDoubleOrNull() ?: 0.0
                    val startY = content[1].toDoubleOrNull() ?: 0.0
                    val startH = content[2].toDoubleOrNull() ?: 0.0
                    PedroComponent.follower.pose = Pose(startX, startY, startH)
                }
            } catch (e: Exception) {
                // File read failed, use default pose
                PedroComponent.follower.pose = Pose(0.0, 0.0, 0.0)
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

        // Start driver control
        driverControlled!!()

        // Intake control
        val intakeMotorDrive = Intake.DriverCommand(
            Gamepads.gamepad1.rightTrigger,
            Gamepads.gamepad1.leftTrigger
        )
        intakeMotorDrive()

        // Shooting (gate + intake)
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

        // ============================================
        // TURRET AUTO-AIM (Triple Fusion)
        // ============================================

        // Hold circle to enable auto-aim with Kalman fusion
        Gamepads.gamepad2.circle whenBecomesTrue {
            // Create and start the fusion aim command
            aimCommand = TripleFusionAim(
                goalX = goalX,
                goalY = goalY,
                poseX = { PedroComponent.follower.pose.x },
                poseY = { PedroComponent.follower.pose.y },
                poseHeading = { PedroComponent.follower.pose.heading }
            )
            aimCommand!!()

            gamepad2.rumble(100)
        } whenBecomesFalse {
            // Stop aiming when released
            Turret.stop()
            aimCommand = null
        }

        // Manual turret control (when not auto-aiming)
        Gamepads.gamepad2.square whenBecomesTrue {
            Turret.Manual {
                Gamepads.gamepad2.rightStickX.get() * 0.5
            }()
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
        telemetry.addData("Loop Time (ms)", runtime - lastRuntime)
        lastRuntime = runtime

        PedroComponent.follower.update()

        // Distance calculations (for flywheel speed, theta, etc.)
        val dx = goalX - x
        val dy = goalY - y
        val dxy = hypot(dx, dy)
        val dxp = dx + vx * distanceToTime(dxy)
        val dyp = dy + vy * distanceToTime(dxy)
        val dxyp = hypot(dxp, dyp)

        // Telemetry
        telemetry.addData("x (inch)", x)
        telemetry.addData("y (inch)", y)
        telemetry.addData("h (radians)", h.inRad)
        telemetry.addData("distanceToGoal", dxy)
        telemetry.addData("Auto Aim", aimCommand != null)
        telemetry.addData("Reset Mode", resetMode)
        telemetry.update()

        PanelsTelemetry.telemetry.addData("CMD", CommandManager.snapshot)
        PanelsTelemetry.telemetry.update()
    }

    override fun onStop() {
        // Save pose to file for next run
        val file = File(Lefile.filePath)
        file.writeText(
            x.toString() + "\n" +
                    y.toString() + "\n" +
                    h.inRad.toString() + "\n"
        )
    }
}
