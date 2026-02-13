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
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech.LimelightAimMT2

/**
 * Turret Limelight Aiming Test
 * Tests turret aiming using Limelight AprilTag detection (MT2)
 * 
 * Controls:
 * - Gamepad2 Circle: Toggle MT2 aiming at speaker
 * - Gamepad2 Square: Toggle MT1 aiming
 * - Gamepad2 Right Stick: Manual turret control
 */
@Configurable
@TeleOp(name = "Turret LL Test", group = "Turret Tests")
class TurretLLTest : NextFTCOpMode() {

    private var mt2AimCommand: LimelightAimMT2? = null
    private var isAimingMT2 = false
    private var mt1AimCommand: org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech.LimelightAim? = null
    private var isAimingMT1 = false

    // Goal position for MT2 aiming
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

        // Toggle MT2 aiming (uses robot pose from AprilTags)
        Gamepads.gamepad2.circle whenBecomesTrue {
            if (isAimingMT2) {
                Turret.stop()
                mt2AimCommand = null
                isAimingMT2 = false
            } else {
                mt2AimCommand = LimelightAimMT2(
                    goalX = goalX,
                    goalY = goalY
                )
                mt2AimCommand!!()
                isAimingMT2 = true
            }
        }

        // Toggle MT1 aiming (no robot pose, just tx)
        Gamepads.gamepad2.square whenBecomesTrue {
            if (isAimingMT1) {
                Turret.stop()
                mt1AimCommand = null
                isAimingMT1 = false
            } else {
                mt1AimCommand = org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech.LimelightAim()
                mt1AimCommand!!()
                isAimingMT1 = true
            }
        }
    }

    override fun onUpdate() {
        CommandManager.cancelAll()

        // Update commands if active
        if (isAimingMT2 && mt2AimCommand != null) {
            mt2AimCommand = LimelightAimMT2(goalX = goalX, goalY = goalY)
            mt2AimCommand!!()
        }
        if (isAimingMT1 && mt1AimCommand != null) {
            mt1AimCommand = org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech.LimelightAim()
            mt1AimCommand!!()
        }

        // Get Limelight data
        val llResult = LLBase.ll.latestResult

        // Telemetry
        PanelsTelemetry.telemetry.addLine("=== TURRET LIMELIGHT TEST ===")
        PanelsTelemetry.telemetry.addData("LL Valid", llResult?.isValid ?: false)
        PanelsTelemetry.telemetry.addData("LL tx", llResult?.tx ?: "null")
        PanelsTelemetry.telemetry.addData("LL ty", llResult?.ty ?: "null")
        
        // MT2 data
        val mt2 = llResult?.botpose_MT2
        PanelsTelemetry.telemetry.addData("MT2 X", mt2?.position?.x ?: "null")
        PanelsTelemetry.telemetry.addData("MT2 Y", mt2?.position?.y ?: "null")
        PanelsTelemetry.telemetry.addData("MT2 Yaw", mt2?.orientation?.yaw ?: "null")
        
        // MT1 data
        val mt1 = llResult?.botpose
        PanelsTelemetry.telemetry.addData("MT1 X", mt1?.position?.x ?: "null")
        PanelsTelemetry.telemetry.addData("MT1 Y", mt1?.position?.y ?: "null")
        PanelsTelemetry.telemetry.addData("MT1 Yaw", mt1?.orientation?.yaw ?: "null")
        
        PanelsTelemetry.telemetry.addData("Turret Angle (deg)", Math.toDegrees(Turret.turretYaw.inRad))
        PanelsTelemetry.telemetry.addData("Turret Encoder", Turret.motor.currentPosition)
        PanelsTelemetry.telemetry.addData("Aiming MT2", isAimingMT2)
        PanelsTelemetry.telemetry.addData("Aiming MT1", isAimingMT1)
        PanelsTelemetry.telemetry.addData("Controller Goal", Turret.controller.goal.position)
        PanelsTelemetry.telemetry.addData("Controller Output", Turret.controller.calculate(Turret.motor.state))
        PanelsTelemetry.telemetry.update()
    }
}
