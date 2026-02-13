package org.firstinspires.ftc.teamcode.subsystems.outtake

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry.telemetry
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.FeedbackType
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.hardware.controllable.RunToState
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import java.util.function.Supplier


@Configurable
object ShooterSubsystem : Subsystem {
    private val motor1 = MotorEx("Fly1");
    private val motor2 = MotorEx("Fly2").reversed();

    @JvmField var ffCoefficients = BasicFeedforwardParameters(0.0, 0.0, 0.75);
    @JvmField var pidCoefficients = PIDCoefficients(0.016, 0.0, 0.0)
    @JvmField var shootingCoefficients = PIDCoefficients(1.0, 0.0, 0.0)

    private val flyWheelController: ControlSystem;
    private val shootingController: ControlSystem;

    init {
        flyWheelController = controlSystem {
            basicFF(ffCoefficients)
            velPid(pidCoefficients)
        }

        // todo: tune shooting controller
        shootingController = controlSystem {
            basicFF(ffCoefficients)
            velPid(shootingCoefficients)
        }
    }

    fun setMotorPowers(power: Double) {
        motor1.power = power;
        motor2.power = power;
    }

    fun setControllerGoals(velocity: Double) {
        flyWheelController.goal = KineticState(velocity=velocity);
        shootingController.goal = KineticState(velocity=velocity);
    }

    class On(private val velocity: Double): Command() {
        override val isDone = true;

        override fun start() {
            setControllerGoals(velocity)
        }
    }

    var off = On(0.0);

    // use this to find distance on field with right velo after pid tuning
    class Manual(
        private val shooterPower: Supplier<Double>
    ) : Command() { override val isDone = false; override fun update() {
            setMotorPowers(shooterPower.get())
        } }

    class AutoAim(
        private val dxy: Double, // figure out what this is
        private val powerByDistance: (Double) -> Double,  // get by running curve of best fit on collected data
    ) : Command() {
        override val isDone = true;

        init {
            requires(ShooterSubsystem)
        }

        override fun start() {
            setControllerGoals(powerByDistance(dxy))
        }
    }

    var isShooting = false;
    var lastPos = 0.0;
    var elapsedTime: ElapsedTime = ElapsedTime();
    override fun periodic() {
        val power = (if (isShooting) shootingController else flyWheelController).calculate(
            motor2.state.times(-1.0)
        ).coerceIn(0.0, 1.0);

        setMotorPowers(power);

        val measuredVel = (motor2.currentPosition - lastPos)/elapsedTime.time();
        lastPos = motor2.currentPosition;
        elapsedTime.reset()

        telemetry.addData("power", power)

        telemetry.addData("vel measured", measuredVel)
        telemetry.addData("vel est", flyWheelController.lastMeasurement.velocity)
        telemetry.addData("vel ref", flyWheelController.reference.velocity)
        telemetry.addData("vel goal", flyWheelController.goal.velocity)

        telemetry.addData("pos measured 1", motor1.currentPosition)
        telemetry.addData("pos measured 2", motor2.currentPosition)
        telemetry.addData("pos est", -flyWheelController.lastMeasurement.position)
        telemetry.addData("pos ref", flyWheelController.reference.position)
    }
}