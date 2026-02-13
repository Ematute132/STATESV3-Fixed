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

@Configurable
@TeleOp(name = "Gate Test", group = "Base Subsystem Tests")
class GateTest: NextFTCOpMode() {
    init{
        addComponents(
            SubsystemComponent(Gate),
            BindingsComponent, 
            BulkReadComponent
        )
    }

    override fun onStartButtonPressed() {
        Gamepads.gamepad1.a whenBecomesTrue Gate.open whenBecomesFalse Gate.close
    }

    override fun onUpdate() {
        PanelsTelemetry.telemetry.addLine("Gate Test Running")
    }
}
