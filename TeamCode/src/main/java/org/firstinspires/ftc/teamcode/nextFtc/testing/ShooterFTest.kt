package org.firstinspires.ftc.teamcode.nextFtc.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.FlyWheel

@Configurable
@TeleOp(name = "Shooter F Test", group = "Base Subsystem Tests")
class ShooterFTest : NextFTCOpMode() {
    companion object {
        @JvmField var speed = 0.0;
    }

    init {
        addComponents(
            SubsystemComponent(FlyWheel),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onUpdate() {
        CommandManager.cancelAll()
        FlyWheel.setVelocity(speed * 2000)  // Convert 0-1 to velocity
        PanelsTelemetry.telemetry.update();
    }
}
