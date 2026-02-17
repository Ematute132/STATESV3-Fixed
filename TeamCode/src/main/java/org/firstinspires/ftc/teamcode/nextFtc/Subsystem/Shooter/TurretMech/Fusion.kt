package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech

import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.commands.Command
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.LL.LLBase
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.Turret
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2

/**
 * Triple-Source Kalman Fusion: Odometry + MT1 + MT2
 *
 * The "big boy" filter that uses everything:
 * - Odometry: Primary source (most accurate, always available)
 * - MT2: Drift correction (uses robot orientation feed)
 * - MT1: Secondary drift correction (fallback)
 *
 * Trust hierarchy: Odo > MT2 > MT1
 *
 * Vision measurements gently nudge the estimate to correct for
 * long-term odometry drift, but odo drives the show.
 */
class TripleFusionAim(
    private val goalX: Double,
    private val goalY: Double,
    private val poseX: Supplier<Double>,
    private val poseY: Supplier<Double>,
    private val poseHeading: Supplier<Double>,  // Returns RADIANS
    private val ofsTurret: Angle = 0.0.rad
) : Command() {

    // ============================================
    // KALMAN FILTER STATE
    // ============================================

    private var kfAngle = 0.0
    private var kfP = 1.0

    // ============================================
    // KALMAN FILTER NOISE - TUNE THESE!
    // ============================================
    // Process noise (odometry model) - LOW because odo is most accurate
    @JvmField var processNoiseQ = 0.005
    
    // Measurement noise - HIGHER because vision is less trusted
    @JvmField var mt2NoiseR = 0.15    
    @JvmField var mt1NoiseR = 0.25

    // Adaptive noise scaling
    @JvmField var rotationNoiseScale = 1.5
    @JvmField var dropoutNoiseScale = 1.5
    @JvmField var dropoutThreshold = 10

    // Tracking
    private var visionDropoutCounter = 0
    private var lastMT1Update = false
    private var lastMT2Update = false
    private var lastOdoUpdate = false

    override val isDone = false

    override fun start() {
        Turret.currentState = Turret.State.KALMAN_AIM
        Turret.registerCommand(this)

        kfAngle = Turret.turretYaw
        kfP = 1.0
        visionDropoutCounter = 0
    }

    override fun update() {
        lastMT1Update = false
        lastMT2Update = false
        lastOdoUpdate = false

        // ============================================
        // ADAPTIVE NOISE CALCULATION
        // ============================================

        val angVel = abs(Turret.robotAngularVelocity)
        val adaptiveQ = processNoiseQ * (1.0 + angVel * rotationNoiseScale)

        val dropoutMultiplier = if (visionDropoutCounter > dropoutThreshold) dropoutNoiseScale else 1.0
        val adaptiveMT2R = mt2NoiseR * dropoutMultiplier
        val adaptiveMT1R = mt1NoiseR * dropoutMultiplier

        // ============================================
        // PREDICT STEP (Odometry)
        // ============================================

        val odoAngle = computeOdoAngle()
        if (odoAngle != null) {
            kalmanPredict(odoAngle, adaptiveQ)
            lastOdoUpdate = true
        } else {
            kfP += adaptiveQ
        }

        // ============================================
        // UPDATE STEP (Vision - MT2 preferred, MT1 fallback)
        // ============================================

        val mt2Angle = computeMT2Angle()
        val mt1Angle = computeMT1Angle()

        var hadVisionUpdate = false

        // Apply MT2 first (most accurate)
        if (mt2Angle != null) {
            kalmanUpdate(mt2Angle, adaptiveMT2R)
            lastMT2Update = true
            hadVisionUpdate = true
        }

        // Apply MT1 as additional measurement
        if (mt1Angle != null) {
            kalmanUpdate(mt1Angle, adaptiveMT1R)
            lastMT1Update = true
            hadVisionUpdate = true
        }

        if (hadVisionUpdate) {
            visionDropoutCounter = 0
        } else {
            visionDropoutCounter++
        }

        // ============================================
        // APPLY FUSED ESTIMATE
        // ============================================

        val fusedAngle = Turret.clampAngle(kfAngle.rad)
        Turret.setTargetAngle(fusedAngle, compensateVelocity = true)

        // Telemetry
        PanelsTelemetry.telemetry.addData("Triple KF Angle (deg)", Math.toDegrees(kfAngle))
        PanelsTelemetry.telemetry.addData("Triple KF Covariance", "%.4f".format(kfP))
        PanelsTelemetry.telemetry.addData("Triple KF MT2", lastMT2Update)
        PanelsTelemetry.telemetry.addData("Triple KF MT1", lastMT1Update)
        PanelsTelemetry.telemetry.addData("Triple KF Odo", lastOdoUpdate)
        PanelsTelemetry.telemetry.addData("Vision Dropout", visionDropoutCounter)
    }

    override fun stop(interrupted: Boolean) {
        if (!interrupted) Turret.currentState = Turret.State.IDLE
    }

    // ============================================
    // KALMAN FILTER MATH
    // ============================================

    private fun kalmanPredict(predictedAngle: Double, processNoise: Double) {
        kfAngle = predictedAngle
        kfP += processNoise
    }

    private fun kalmanUpdate(measuredAngle: Double, measurementNoise: Double) {
        var innovation = measuredAngle - kfAngle
        if (innovation > PI) innovation -= 2 * PI
        if (innovation < -PI) innovation += 2 * PI

        val S = kfP + measurementNoise
        val K = kfP / S

        kfAngle += K * innovation
        kfP = (1.0 - K) * kfP
    }

    // ============================================
    // ANGLE COMPUTATIONS
    // ============================================

    /**
     * Compute angle to goal using odometry
     * poseHeading is already in RADIANS
     */
    private fun computeOdoAngle(): Double? {
        val x = poseX.get()
        val y = poseY.get()
        val h = poseHeading.get()

        val deltaX = goalX - x
        val deltaY = goalY - y
        val fieldAngle = atan2(deltaY, deltaX)

        return Turret.normalizeAngle(fieldAngle - h + ofsTurret.inRad)
    }

    /**
     * Compute angle using MegaTag2 (most accurate)
     */
    private fun computeMT2Angle(): Double? {
        val result = LLBase.ll.latestResult
        if (result == null || !result.isValid) return null

        val mt2 = result.botpose_MT2 ?: return null

        val robotX = mt2.position.x
        val robotY = mt2.position.y
        if (robotX == 0.0 && robotY == 0.0) return null

        val robotYaw = Math.toRadians(mt2.orientation.yaw)

        val deltaX = goalX - robotX
        val deltaY = goalY - robotY
        val fieldAngle = atan2(deltaY, deltaX)

        return Turret.normalizeAngle(fieldAngle - robotYaw + ofsTurret.inRad)
    }

    /**
     * Compute angle using MegaTag1 (fallback)
     */
    private fun computeMT1Angle(): Double? {
        val result = LLBase.ll.latestResult
        if (result == null || !result.isValid) return null

        val mt1 = result.botpose ?: return null

        val robotX = mt1.position.x
        val robotY = mt1.position.y
        if (robotX == 0.0 && robotY == 0.0) return null

        val robotYaw = Math.toRadians(mt1.orientation.yaw)

        val deltaX = goalX - robotX
        val deltaY = goalY - robotY
        val fieldAngle = atan2(deltaY, deltaX)

        return Turret.normalizeAngle(fieldAngle - robotYaw + ofsTurret.inRad)
    }
}


/**
 * Simplified version - prioritizes MT2, falls back to MT1, then odo-only
 */
class TripleFusionAimSimple(
    private val goalX: Double,
    private val goalY: Double,
    private val poseX: Supplier<Double>,
    private val poseY: Supplier<Double>,
    private val poseHeading: Supplier<Double>,
    private val ofsTurret: Angle = 0.0.rad
) : Command() {

    private var kfAngle = 0.0
    private var kfP = 1.0

    companion object {
        @JvmField var processNoiseQ = 0.005
        @JvmField var mt2NoiseR = 0.15
        @JvmField var mt1NoiseR = 0.25
    }

    private var sourceUsed = "none"

    override val isDone = false

    override fun start() {
        Turret.currentState = Turret.State.KALMAN_AIM
        Turret.registerCommand(this)
        kfAngle = Turret.turretYaw
        kfP = 1.0
    }

    override fun update() {
        // Predict from odometry
        val odoAngle = computeOdoAngle()
        if (odoAngle != null) {
            kfAngle = odoAngle
            kfP += processNoiseQ
        } else {
            kfP += processNoiseQ
        }

        // Update from vision (priority: MT2 > MT1)
        val mt2Angle = computeMT2Angle()
        val mt1Angle = computeMT1Angle()

        when {
            mt2Angle != null -> {
                applyUpdate(mt2Angle, mt2NoiseR)
                sourceUsed = "MT2"
            }
            mt1Angle != null -> {
                applyUpdate(mt1Angle, mt1NoiseR)
                sourceUsed = "MT1"
            }
            else -> {
                sourceUsed = "Odo-only"
            }
        }

        val fusedAngle = Turret.clampAngle(kfAngle.rad)
        Turret.setTargetAngle(fusedAngle, compensateVelocity = true)

        PanelsTelemetry.telemetry.addData("Simple KF Angle", "%.1f".format(Math.toDegrees(kfAngle)))
        PanelsTelemetry.telemetry.addData("Simple KF Source", sourceUsed)
        PanelsTelemetry.telemetry.addData("Simple KF P", "%.4f".format(kfP))
    }

    override fun stop(interrupted: Boolean) {
        if (!interrupted) Turret.currentState = Turret.State.IDLE
    }

    private fun applyUpdate(measuredAngle: Double, noise: Double) {
        var y = measuredAngle - kfAngle
        if (y > PI) y -= 2 * PI
        if (y < -PI) y += 2 * PI

        val S = kfP + noise
        val K = kfP / S
        kfAngle += K * y
        kfP = (1.0 - K) * kfP
    }

    private fun computeOdoAngle(): Double? {
        val x = poseX.get()
        val y = poseY.get()
        val h = poseHeading.get()

        val deltaX = goalX - x
        val deltaY = goalY - y
        val fieldAngle = atan2(deltaY, deltaX)

        return Turret.normalizeAngle(fieldAngle - h + ofsTurret.inRad)
    }

    private fun computeMT2Angle(): Double? {
        val result = LLBase.ll.latestResult ?: return null
        if (!result.isValid) return null
        val mt2 = result.botpose_MT2 ?: return null
        if (mt2.position.x == 0.0 && mt2.position.y == 0.0) return null

        val deltaX = goalX - mt2.position.x
        val deltaY = goalY - mt2.position.y
        val fieldAngle = atan2(deltaY, deltaX)
        val robotYaw = Math.toRadians(mt2.orientation.yaw)

        return Turret.normalizeAngle(fieldAngle - robotYaw + ofsTurret.inRad)
    }

    private fun computeMT1Angle(): Double? {
        val result = LLBase.ll.latestResult ?: return null
        if (!result.isValid) return null
        val mt1 = result.botpose ?: return null
        if (mt1.position.x == 0.0 && mt1.position.y == 0.0) return null

        val deltaX = goalX - mt1.position.x
        val deltaY = goalY - mt1.position.y
        val fieldAngle = atan2(deltaY, deltaX)
        val robotYaw = Math.toRadians(mt1.orientation.yaw)

        return Turret.normalizeAngle(fieldAngle - robotYaw + ofsTurret.inRad)
    }
}
