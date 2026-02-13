package org.firstinspires.ftc.teamcode.Subsystem.Shooter

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

object Hood: Subsystem{

    @JvmField var DOWN = 0.0;  // todo: tune
    @JvmField var CLOSE = 0.0;  // todo: tune
    @JvmField var MID = 0.0;  // todo: tune
    @JvmField var FAR = 1.0;

    private val servo = ServoEx("hood");

    val mid = SetPosition(servo,MID)
    val down = SetPosition(servo,DOWN)
    val close = SetPosition(servo,CLOSE)
    val far = SetPosition(servo, FAR)
    // got to work on the auto alignment based off of distance

}