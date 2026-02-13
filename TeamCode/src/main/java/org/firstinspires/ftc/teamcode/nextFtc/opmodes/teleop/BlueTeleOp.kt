package org.firstinspires.ftc.teamcode.nextFtc.opmodes.teleop

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad

/**
 * Blue Alliance TeleOp
 * Uses Blue side field coordinates and goal position
 */
@Configurable
@TeleOp(name = "Blue TeleOp", group = "TeleOp")
class BlueTeleOp : TeleOpBase(
    isBlue = true,
    goalX = -72.0,      // Blue alliance goal X position
    goalY = 0.0,        // Blue alliance goal Y position
    resetModeParams = ResetModeParams(
        x = -72.0,       // Starting X for blue
        y = -72.0,       // Starting Y for blue
        h = 0.0.deg      // Starting heading
    ),
    resetModePhiAngle = 0.0.deg,
    distanceToVelocity = { distance ->
        // Convert distance to flywheel velocity
        // TODO: TUNE THIS FUNCTION
        when {
            distance < 20.0 -> 1000.0
            distance < 40.0 -> 1250.0
            distance < 60.0 -> 1500.0
            else -> 1750.0
        }
    },
    distanceToTheta = { distance ->
        // Convert distance to hood/flywheel angle
        // TODO: TUNE THIS FUNCTION
        when {
            distance < 20.0 -> 0.3.rad
            distance < 40.0 -> 0.5.rad
            distance < 60.0 -> 0.7.rad
            else -> 0.9.rad
        }
    },
    distanceToTime = { distance ->
        // Time offset for moving target compensation
        // TODO: TUNE THIS VALUE
        distance * 0.05
    }
)
