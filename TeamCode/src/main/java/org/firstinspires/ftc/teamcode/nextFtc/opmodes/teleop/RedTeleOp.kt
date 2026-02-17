package org.firstinspires.ftc.teamcode.nextFtc.opmodes.teleop

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad

/**
 * Red Alliance TeleOp
 */
@Configurable
@TeleOp(name = "Red TeleOp", group = "TeleOp")
class RedTeleOp : TeleOpBase(
    isBlue = false,
    goalX = 144.0,       // Red side goal
    goalY = 144.0,
    resetModeParams = ResetModeParams(
        x = 72.0,
        y = -72.0,
        h = 0.0.deg
    ),
    resetModePhiAngle = 0.0.deg,
    distanceToVelocity = { distance ->
        when {
            distance < 20.0 -> 1000.0
            distance < 40.0 -> 1250.0
            distance < 60.0 -> 1500.0
            else -> 1750.0
        }
    },
    distanceToTheta = { distance ->
        when {
            distance < 20.0 -> 0.3.rad
            distance < 40.0 -> 0.5.rad
            distance < 60.0 -> 0.7.rad
            else -> 0.9.rad
        }
    },
    distanceToTime = { distance ->
        distance * 0.05
    }
)
