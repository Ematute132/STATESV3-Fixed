package org.firstinspires.ftc.teamcode.opmodes.testing.baseSubsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.ftc.NextFTCOpMode
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

//
//    init {
//        addComponents(
//            SubsystemComponent(ShooterSubsystem),
//            BulkReadComponent,
//            BindingsComponent
//        )
//    }

    override fun onUpdate() {
        motor1.power = power1;
        motor2.power = power2;
    }



}