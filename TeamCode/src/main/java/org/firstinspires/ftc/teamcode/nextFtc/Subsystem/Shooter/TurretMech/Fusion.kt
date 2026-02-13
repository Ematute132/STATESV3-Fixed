package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech

import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.commands.Command
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.LL.LLBase
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
 *
 * Usage:
 * ```kotlin
 * val aim = TripleFusionAim(
 *     goalX = 72.0,
 *     goalY = 0.0,
 *     poseX = { PedroComponent.follower.pose.x },
 *     poseY = { PedroComponent.follower.pose.y },
 *     poseHeading = { PedroComponent.follower.pose.heading }
 * )
 * CommandManager.scheduleCommand(aim)
 * ```
 */
class TripleFusionAim(
    private val goalX: Double,
    private val goalY: Double,
    private val poseX: Supplier<Double>,
    private val poseY: Supplier<Double>,
    private val poseHeading: Supplier<Double>,  // Radians
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
    // HOW TO TUNE:
    // 1. Watch telemetry during operation
    // 2. If KF jumps around too much -> increase noise (R values)
    // 3. If KF is too slow to respond -> decrease noise (R values)
    // 4. Process noise (Q) affects how much we trust odometry between updates
    // ============================================
    
    // Process noise (odometry model) - LOW because odo is most accurate
    @JvmField var processNoiseQ = 0.005  // TODO: TUNE - Increase if KF lags behind odo
    
    // Measurement noise - HIGHER because vision is less trusted
    // Vision is used for drift correction, not primary tracking
    @JvmField var mt2NoiseR = 0.15    // TODO: TUNE - MT2 trust level (0.05 = high trust, 0.5 = low trust)
    @JvmField var mt1NoiseR = 0.25    // TODO: TUNE - MT1 trust level (usually higher than MT2)

    // ============================================
    // ADAPTIVE NOISE SCALING - OPTIONAL!
    // ============================================
    // These adjust noise dynamically based on conditions
    // Start with 1.0 (no scaling) and tune if needed
    // ============================================
    
    @JvmField var rotationNoiseScale = 1.5   // TODO: TUNE - Scale Q when robot rotating fast
    @JvmField var dropoutNoiseScale = 1.5     // TODO: TUNE - Scale R after vision dropout
    @JvmField var dropoutThreshold = 10      // Frames before increasing noise

    // Tracking
    private var visionDropoutCounter = 0
    private var lastMT1Update = false
    private var lastMT2Update = false
    private var lastOdoUpdate = false

    override val isDone = false

    override fun start() {
        Turret.currentState = Turret.State.KALMAN_AIM
        Turret.registerCommand(this)

        kfAngle = Turret.turretYaw.inRad
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

        // Increase process noise when robot is rotating fast
        val angVel = abs(Turret.robotAngularVelocity)
        val adaptiveQ = processNoiseQ * (1.0 + angVel * rotationNoiseScale)

        // Increase measurement noise after vision dropouts
        val dropoutMultiplier = if (visionDropoutCounter > dropoutThreshold) dropoutNoiseScale else 1.0
        val adaptiveMT2R = mt2NoiseR * dropoutMultiplier
        val adaptiveMT1R = mt1NoiseR * dropoutMultiplier

        // ============================================
        // PREDICT STEP (Odometry)
        // ============================================

        val odoAngle = computeOdoAngle()
        if (odoAngle != null) {
            kalmanPredict(odoAngle.inRad, adaptiveQ)
            lastOdoUpdate = true
        } else {
            // No odometry - just increase uncertainty
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
            kalmanUpdate(mt2Angle.inRad, adaptiveMT2R)
            lastMT2Update = true
            hadVisionUpdate = true
        }

        // Apply MT1 as additional measurement (if available)
        // This gives us redundancy and can improve estimate even when MT2 is present
        if (mt1Angle != null) {
            kalmanUpdate(mt1Angle.inRad, adaptiveMT1R)
            lastMT1Update = true
            hadVisionUpdate = true
        }

        // Track dropouts
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
        PanelsTelemetry.telemetry.addData("Triple KF Covariance", kfP)
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
        // Innovation with angle wrapping
        var innovation = measuredAngle - kfAngle
        if (innovation > PI) innovation -= 2 * PI
        if (innovation < -PI) innovation += 2 * PI

        // Kalman gain
        val S = kfP + measurementNoise
        val K = kfP / S

        // State update
        kfAngle += K * innovation

        // Covariance update
        kfP = (1.0 - K) * kfP
    }

    // ============================================
    // ANGLE COMPUTATIONS
    // ============================================

    /**
     * Compute angle to goal using Pedro odometry
     */
    private fun computeOdoAngle(): Angle? {
        val x = poseX.get()
        val y = poseY.get()
        val h = poseHeading.get()

        val deltaX = goalX - x
        val deltaY = goalY - y
        val fieldAngle = atan2(deltaY, deltaX)

        // Handle heading - if > 2*PI, assume degrees
        val robotHeading = if (abs(h) > 2.0 * PI) Math.toRadians(h) else h

        return Turret.normalizeAngle((fieldAngle - robotHeading).rad + ofsTurret)
    }

    /**
     * Compute angle using MegaTag2 (most accurate - uses robot orientation)
     */
    private fun computeMT2Angle(): Angle? {
        val result = LLBase.ll.latestResult
        if (result == null || !result.isValid) return null

        val mt2 = result.botpose_MT2 ?: return null

        // Check if pose is valid (not all zeros)
        val robotX = mt2.position.x
        val robotY = mt2.position.y
        if (robotX == 0.0 && robotY == 0.0) return null

        val robotYaw = Math.toRadians(mt2.orientation.yaw)

        val deltaX = goalX - robotX
        val deltaY = goalY - robotY
        val fieldAngle = atan2(deltaY, deltaX)

        return Turret.normalizeAngle((fieldAngle - robotYaw).rad + ofsTurret)
    }

    /**
     * Compute angle using MegaTag1 (fallback - no orientation input)
     */
    private fun computeMT1Angle(): Angle? {
        val result = LLBase.ll.latestResult
        if (result == null || !result.isValid) return null

        val mt1 = result.botpose ?: return null

        // Check if pose is valid
        val robotX = mt1.position.x
        val robotY = mt1.position.y
        if (robotX == 0.0 && robotY == 0.0) return null

        val robotYaw = Math.toRadians(mt1.orientation.yaw)

        val deltaX = goalX - robotX
        val deltaY = goalY - robotY
        val fieldAngle = atan2(deltaY, deltaX)

        return Turret.normalizeAngle((fieldAngle - robotYaw).rad + ofsTurret)
    }
}


/**
 * Simplified version that prioritizes MT2, falls back to MT1, then odo-only.
 * Only applies ONE vision measurement per cycle (no double-update).
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
        @JvmField var processNoiseQ = 0.005  // Low - odo is king
        @JvmField var mt2NoiseR = 0.15       // Vision just for drift correction
        @JvmField var mt1NoiseR = 0.25
    }

    private var sourceUsed = "none"

    override val isDone = false

    override fun start() {
        Turret.currentState = Turret.State.KALMAN_AIM
        Turret.registerCommand(this)
        kfAngle = Turret.turretYaw.inRad
        kfP = 1.0
    }

    override fun update() {
        // Predict from odometry
        val odoAngle = computeOdoAngle()
        if (odoAngle != null) {
            kfAngle = odoAngle.inRad
            kfP += processNoiseQ
        } else {
            kfP += processNoiseQ
        }

        // Update from vision (priority: MT2 > MT1)
        val mt2Angle = computeMT2Angle()
        val mt1Angle = computeMT1Angle()

        when {
            mt2Angle != null -> {
                applyUpdate(mt2Angle.inRad, mt2NoiseR)
                sourceUsed = "MT2"
            }
            mt1Angle != null -> {
                applyUpdate(mt1Angle.inRad, mt1NoiseR)
                sourceUsed = "MT1"
            }
            else -> {
                sourceUsed = "Odo-only"
            }
        }

        val fusedAngle = Turret.clampAngle(kfAngle.rad)
        Turret.setTargetAngle(fusedAngle, compensateVelocity = true)

        PanelsTelemetry.telemetry.addData("Simple KF Angle", Math.toDegrees(kfAngle))
        PanelsTelemetry.telemetry.addData("Simple KF Source", sourceUsed)
        PanelsTelemetry.telemetry.addData("Simple KF P", kfP)
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

    private fun computeOdoAngle(): Angle? {
        val x = poseX.get()
        val y = poseY.get()
        val h = poseHeading.get()

        val deltaX = goalX - x
        val deltaY = goalY - y
        val fieldAngle = atan2(deltaY, deltaX)
        val robotHeading = if (abs(h) > 2.0 * PI) Math.toRadians(h) else h

        return Turret.normalizeAngle((fieldAngle - robotHeading).rad + ofsTurret)
    }

    private fun computeMT2Angle(): Angle? {
        val result = LLBase.ll.latestResult ?: return null
        if (!result.isValid) return null
        val mt2 = result.botpose_MT2 ?: return null
        if (mt2.position.x == 0.0 && mt2.position.y == 0.0) return null

        val deltaX = goalX - mt2.position.x
        val deltaY = goalY - mt2.position.y
        val fieldAngle = atan2(deltaY, deltaX)
        val robotYaw = Math.toRadians(mt2.orientation.yaw)

        return Turret.normalizeAngle((fieldAngle - robotYaw).rad + ofsTurret)
    }

    private fun computeMT1Angle(): Angle? {
        val result = LLBase.ll.latestResult ?: return null
        if (!result.isValid) return null
        val mt1 = result.botpose ?: return null
        if (mt1.position.x == 0.0 && mt1.position.y == 0.0) return null

        val deltaX = goalX - mt1.position.x
        val deltaY = goalY - mt1.position.y
        val fieldAngle = atan2(deltaY, deltaX)
        val robotYaw = Math.toRadians(mt1.orientation.yaw)

        return Turret.normalizeAngle((fieldAngle - robotYaw).rad + ofsTurret)
    }
}