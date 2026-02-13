package org.firstinspires.ftc.teamcode.nextFtc.testing

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.hardware.impl.MotorEx

@Configurable
@TeleOp(name = "Shooter 2 Test", group = "Base Subsystem Tests")
class ShooterTest2 : NextFTCOpMode() {
    private val motor2 = MotorEx("Fly2");

    companion object {
        @JvmField var power = 0.0;
    }

    init {
        addComponents(
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        motor2.power = power;
    }
}
