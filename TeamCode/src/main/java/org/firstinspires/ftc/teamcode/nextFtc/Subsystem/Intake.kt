package org.firstinspires.ftc.teamcode.Subsystem

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import java.util.function.Supplier

@Configurable
object Intake : Subsystem {
    private val motor = MotorEx("intake").reversed()

    class On(power: Double) : InstantCommand({ motor.power = power })
    var intake = SetPower(motor, 1.0);
    var reverse = SetPower(motor, -1.0);
    var off = SetPower(motor, 0.0);


    class DriverCommandDefaultOn(
        private val outPower: Supplier<Double>,
    ) : Command() {
        override val isDone = false;

        init {
            setInterruptible(true);
            setName("Intake Drive")
        }

        override fun start() {
            motor.power = 0.0;
        }

        override fun update() {
            motor.power = 1.0 - 2.0 * outPower.get();
        }
    }



    class DriverCommand(
        private val inPower: Supplier<Double>,
        private val outPower: Supplier<Double>
    ) : Command() {
        override val isDone = false;

        init {
            setInterruptible(true);
            setName("Intake Drive")
        }

        override fun start() {
            motor.power = 0.0;
        }

        override fun update() {
            motor.power = inPower.get() - outPower.get();
        }
    }
}