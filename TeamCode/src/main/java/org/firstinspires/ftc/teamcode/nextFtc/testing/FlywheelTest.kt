package org.firstinspires.ftc.teamcode.nextFtc.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.FlyWheel

/**
 * Flywheel Velocity Test
 * Tests and tunes flywheel velocity control
 * 
 * Controls:
 * - Gamepad2 Right Trigger: Set velocity (0-1 scales to max velocity)
 * - Gamepad2 Left Trigger: Set velocity in reverse
 * - Gamepad2 X: Run preset CLOSE velocity
 * - Gamepad2 Y: Run preset MID velocity
 * - Gamepad2 B: Run preset FAR velocity
 * - Gamepad2 A: Stop flywheel
 */
@Configurable
@TeleOp(name = "Flywheel Test", group = "Shooter Tests")
class FlywheelTest : NextFTCOpMode() {

    companion object {
        @JvmField var velocityMultiplier = 0.5  // 0-1 scale for velocity testing
    }

    init {
        addComponents(
            SubsystemComponent(FlyWheel),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        // Manual velocity control with triggers
        Gamepads.gamepad2.rightTrigger.whenNotZero { value ->
            FlyWheel.setVelocity(value * 2000)  // Scale to max ~2000 RPM
        }

        Gamepads.gamepad2.leftTrigger.whenNotZero { value ->
            FlyWheel.setVelocity(-value * 500)  // Reverse at low speed
        }

        // Preset velocities
        Gamepads.gamepad2.x whenBecomesTrue {
            FlyWheel.runClose()
        }
        Gamepads.gamepad2.y whenBecomesTrue {
            FlyWheel.runMid()
        }
        Gamepads.gamepad2.b whenBecomesTrue {
            FlyWheel.runFar()
        }
        Gamepads.gamepad2.a whenBecomesTrue {
            FlyWheel.setVelocity(0.0)
        }
    }

    override fun onUpdate() {
        CommandManager.cancelAll()

        // Telemetry
        PanelsTelemetry.telemetry.addLine("=== FLYWHEEL TEST ===")
        
        // Motor states
        PanelsTelemetry.telemetry.addData("Fly1 Position", FlyWheel.controller.state.position)
        PanelsTelemetry.telemetry.addData("Fly1 Velocity", FlyWheel.controller.state.velocity)
        PanelsTelemetry.telemetry.addData("Fly1 Power", FlyWheel.controller.state.power)
        
        // Goal
        PanelsTelemetry.telemetry.addData("Target Velocity", FlyWheel.targetVelocity)
        PanelsTelemetry.telemetry.addData("Velocity Multiplier", velocityMultiplier)
        
        // Controller status
        PanelsTelemetry.telemetry.addData("Controller Goal Pos", FlyWheel.controller.goal.position)
        PanelsTelemetry.telemetry.addData("Controller Goal Vel", FlyWheel.controller.goal.velocity)
        PanelsTelemetry.telemetry.addData("Is Running", FlyWheel.isRunning)
        
        // Tuning values
        PanelsTelemetry.telemetry.addData("kV (FF)", FlyWheel.ffCoefficients.kV)
        PanelsTelemetry.telemetry.addData("kA (FF)", FlyWheel.ffCoefficients.kA)
        PanelsTelemetry.telemetry.addData("kS (FF)", FlyWheel.ffCoefficients.kS)
        PanelsTelemetry.telemetry.addData("kP (PID)", FlyWheel.pidCoefficients.kP)
        PanelsTelemetry.telemetry.addData("kI (PID)", FlyWheel.pidCoefficients.kI)
        PanelsTelemetry.telemetry.addData("kD (PID)", FlyWheel.pidCoefficients.kD)
        
        // Comparison
        val error = FlyWheel.targetVelocity - FlyWheel.controller.state.velocity
        PanelsTelemetry.telemetry.addData("Velocity Error", error)
        PanelsTelemetry.telemetry.addData("Error %", if (FlyWheel.targetVelocity > 0) error / FlyWheel.targetVelocity * 100 else 0.0)
        
        PanelsTelemetry.telemetry.update()
    }
}
