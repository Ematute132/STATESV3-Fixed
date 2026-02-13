package org.firstinspires.ftc.teamcode.nextFtc.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Gate
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.Hood

@Configurable
@TeleOp(name = "Hood Test", group = "Base Subsystem Tests")
class HoodTest: NextFTCOpMode() {
    init{
        addComponents(
            SubsystemComponent(Hood),
            BindingsComponent, 
            BulkReadComponent
        )
    }

    override fun onStartButtonPressed() {
        Gamepads.gamepad1.circle whenBecomesTrue Hood.far whenBecomesFalse Hood.down
        Gamepads.gamepad1.square whenBecomesTrue Hood.mid whenBecomesFalse Hood.down
        Gamepads.gamepad1.triangle whenBecomesTrue Hood.close whenBecomesFalse Hood.down
    }

    override fun onUpdate() {
        PanelsTelemetry.telemetry.addLine("Hood Test Running")
    }
}
