package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.LL



import com.bylazar.telemetry.PanelsTelemetry.telemetry
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx



/**
 * Limelight vision subsystem.
 * Handles target detection, distance calculation, and fiducial tracking.
 */
object LLBase: Subsystem{

    lateinit var ll: Limelight3A
    private var isInitialized = false
    val imu = IMUEx("imu", Direction.LEFT, Direction.UP)

    // Fiducial data

    var fiducialCount: Int = 0
        private set
    var fiducialData: String = "No fiducials"
        private set


    override fun initialize() {
        ll = hardwareMap.get(Limelight3A::class.java, "ll")
        ll.pipelineSwitch(0)
        ll.setPollRateHz(100)// This sets how often we ask Limelight for data (100 times per second)
        ll.start()

    }

    override fun periodic() {
        updateFiducialData()
        updateLLPose()
    }



    fun updateFiducialData() {
        val result = ll.latestResult
        if (result != null && result.isValid) {
            val fiducials = result.fiducialResults
            fiducialCount = fiducials.size // Save how many tags we found

            if (fiducials.isNotEmpty()) {
                val sb = StringBuilder() // Just makes it easier to build a long multi-line string
                for (fr in fiducials) {
                    // For each fiducial, we print its ID and position info.
                    sb.append("ID: ${fr.fiducialId}, ")
                    sb.append("X: ${"%.2f".format(fr.targetXDegrees)}°, ")
                    sb.append("Strafe: ${"%.2f".format(fr.robotPoseTargetSpace.position.x)}\n")
                }
                fiducialData = sb.toString().trim()
            } else {
                fiducialData = "No fiducials detected" // No tags were picked up, so we just say that.
            }
        } else {
            // If the Limelight didn’t return a valid result at all, that usually means it has no frame.
            fiducialCount = 0
            fiducialData = "No valid result"
        }
    }


    fun updateLLPose(){
        val result = ll.getLatestResult()
        if (result != null && result.isValid()) {
            val tx = result.tx // How far left or right the target is (degrees)
            val ty = result.ty// How far up or down the target is (degrees)
            val ta = result.ta// How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx)
            telemetry.addData("Target Y", ty)
            telemetry.addData("Target Area", ta)
        } else {
            telemetry.addData("Limelight", "No Targets")
        }
        if (result != null && result.isValid()) {
            val botpose = result.botpose
            if (botpose != null) {
                val x = botpose.getPosition().x
                val y = botpose.getPosition().y
                telemetry.addData("MT1 Location", "($x, $y)")
            }
        }
        val robotYaw: Double = PedroComponent.follower.pose.heading
        ll.updateRobotOrientation(robotYaw)
        if (result != null && result.isValid()) {
            val mt2 = result.getBotpose_MT2()
            if (mt2 != null) {
                val x = mt2.getPosition().x
                val y = mt2.getPosition().y
                telemetry.addData("MT2 Location:", "($x, $y)")
            }
        }

    }
}