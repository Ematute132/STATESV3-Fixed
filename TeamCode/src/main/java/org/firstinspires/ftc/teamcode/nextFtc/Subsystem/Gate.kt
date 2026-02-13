package org.firstinspires.ftc.teamcode.Subsystem

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

object Gate : Subsystem {
    @JvmField var OPEN = 0.0;
    @JvmField var CLOSE = 1.0;

    private val servo = ServoEx("gate");

    val close = SetPosition(servo, CLOSE)
    val open = SetPosition(servo, OPEN)
}