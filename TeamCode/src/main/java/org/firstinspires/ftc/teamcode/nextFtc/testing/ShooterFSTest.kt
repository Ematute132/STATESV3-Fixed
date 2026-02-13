package org.firstinspires.ftc.teamcode.nextFtc.testing

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.hardware.impl.MotorEx

@Configurable
@TeleOp(name = "Shooter FS Test", group = "Base Subsystem Tests")
class ShooterFSTest : NextFTCOpMode() {
    val motor1 = MotorEx("Fly1").reversed();
    val motor2 = MotorEx("Fly2");

    companion object {
        @JvmField var power1 = 0.0;
        @JvmField var power2 = 0.0;
    }

    init {
        addComponents(
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onUpdate() {
        motor1.power = power1;
        motor2.power = power2;
    }
}
