package org.firstinspires.ftc.teamcode.nextFtc.testing

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Intake
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Gate

/**
 * Individual Subsystem Test TeleOp
 * Use this to test each subsystem separately
 * 
 * CONTROLS:
 * Gamepad1:
 * - Left Stick Y: Control intake motor (-1 to 1)
 * - Right Trigger: Intake forward
 * - Left Trigger: Intake reverse
 * - Right Bumper: Open gate
 * - Left Bumper: Close gate
 * - A Button: Stop intake
 * 
 * Gamepad2:
 * - D-Pad Up/Down: Test hood servo position
 * - X Button: Test turret
 */
@Configurable
@TeleOp(name = "Subsystem Test", group = "Testing")
class SubsystemTest : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(
                Intake,
                Gate
            ),
            BindingsComponent
        )
    }

    override fun onInit() {
        telemetry.addData("Status", "Subsystem Test Ready")
        telemetry.addData("Controls", "GP1: Left Stick = Intake, Triggers = In/Out, Bumpers = Gate")
        telemetry.update()
    }

    override fun onStartButtonPressed() {
        telemetry.addData("Status", "Testing Active")
    }

    override fun onUpdate() {
        // === INTAKE TESTS ===
        
        // Left stick Y controls intake
        val stickY = -Gamepads.gamepad1.leftStickY.get()
        if (kotlin.math.abs(stickY) > 0.1) {
            Intake.setPower(stickY)
            telemetry.addData("Intake", "Stick: %.2f".format(stickY))
        }
        
        // Triggers override stick
        val inTrigger = Gamepads.gamepad1.rightTrigger.get()
        val outTrigger = Gamepads.gamepad1.leftTrigger.get()
        
        when {
            inTrigger > 0.1 -> {
                Intake.setPower(inTrigger)
                telemetry.addData("Intake", "In: %.2f".format(inTrigger))
            }
            outTrigger > 0.1 -> {
                Intake.setPower(-outTrigger)
                telemetry.addData("Intake", "Out: %.2f".format(outTrigger))
            }
            kotlin.math.abs(stickY) <= 0.1 -> {
                // Only stop if no input
                if (inTrigger < 0.1 && outTrigger < 0.1 && kotlin.math.abs(stickY) <= 0.1) {
                    // Keep last power (don't force stop)
                }
            }
        }
        
        // A button to stop
        if (Gamepads.gamepad1.a.isPressed) {
            Intake.stop()
            telemetry.addData("Intake", "STOPPED")
        }

        // === GATE TESTS ===
        
        if (Gamepads.gamepad1.rightBumper.isPressed) {
            Gate.open()
            telemetry.addData("Gate", "OPEN")
        } else if (Gamepads.gamepad1.leftBumper.isPressed) {
            Gate.close()
            telemetry.addData("Gate", "CLOSED")
        } else {
            telemetry.addData("Gate", "Idle")
        }

        // === TELEMETRY ===
        telemetry.addData("Intake Power", "%.2f".format(Intake.power))
        telemetry.update()
    }

    override fun onStop() {
        Intake.stop()
        Gate.close()
    }
}
