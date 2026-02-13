package org.firstinspires.ftc.teamcode.nextFtc.testing

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Intake


@Configurable
@TeleOp(name = "Intake Motor Test", group = "Base Subsystem Tests")
class IntakeMotorTest : NextFTCOpMode() {
    companion object {
        @JvmField var power = 0.0;
    }

    init {
        addComponents(
            SubsystemComponent(Intake),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onUpdate() {
        CommandManager.cancelAll()
        Intake.On(power)();
        val lowerMotorDrive = Intake.DriverCommandDefaultOn(
            Gamepads.gamepad2.leftTrigger
     );
        lowerMotorDrive();
    }
}
