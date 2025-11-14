package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

// Note: This class assumes it is being used in an OpMode where `follower.update()` is called
// frequently, and `pushOdomSample()` is called right after, to keep the odometry history fresh.
public class localizer_fusion {
    private final Follower follower;
    private final limelight3A ll;
    private static final double M_TO_IN = 39.37;

    // ring buffer for past odom poses (for latency compensation)
    private static class StampedPose { long tNanos; Pose pose; }
    private final java.util.ArrayDeque<StampedPose> odomHistory = new java.util.ArrayDeque<>();
    private final int HISTORY_CAP = 60; // ~1s if you push at 60Hz

    // --- Tuning knobs ---
    // How much the vision measurement can disagree with odometry before we reject it
    private double linearGateIn = 12.0;   // innovation gate (in)
    private double angularGateDeg = 20.0; // innovation gate (deg)
    // If moving quickly, be more skeptical of vision data
    private double minSpeedForRejectInPerS = 3.0;

    public localizer_fusion(Follower follower, limelight3A ll) {
        this.follower = follower;
        this.ll = ll;
    }

    /**
     * Pushes a new odometry sample to the history buffer.
     * This should be called in your main loop, ideally right after `follower.update()`.
     * @param tNanos The current time from `System.nanoTime()`.
     */
    public void pushOdomSample(long tNanos) {
        Pose p = follower.getPose();
        StampedPose sp = new StampedPose();
        sp.tNanos = tNanos;
        // Make a copy of the pose object
        sp.pose = new Pose(p.getX(), p.getY(), p.getHeading());
        odomHistory.addLast(sp);
        while (odomHistory.size() > HISTORY_CAP) {
            odomHistory.removeFirst();
        }
    }

    /**
     * Finds the odometry pose from history that is closest to the given timestamp.
     * @param targetNanos The timestamp to match.
     * @return The historical odometry pose. Returns the latest pose if history is empty.
     */
    private Pose getOdomAtTime(long targetNanos) {
        StampedPose best = null;
        long bestDt = Long.MAX_VALUE;
        for (StampedPose sp : odomHistory) {
            long dt = Math.abs(sp.tNanos - targetNanos);
            if (dt < bestDt) {
                bestDt = dt;
                best = sp;
            }
        }
        return (best != null) ? best.pose : follower.getPose();
    }

    /**
     * Fetches the latest botpose from the Limelight, validates it, and fuses it with
     * the current odometry to produce a corrected pose.
     * @param nowNanos The current time from `System.nanoTime()`.
     */
    public void updateFromLimelight(long nowNanos) {
        if (ll.limelight == null) return;

        LLResult result = ll.limelight.getLatestResult();
        Pose3D botpose = result.getBotpose();

        // --- Basic Quality Gates ---
        if (botpose == null || !result.isValid()) {
            return; // No valid pose
        }

        // --- Latency Compensation ---
        // Get total latency (capture + pipeline) for accurate timestamping
        double latencyMs = result.getCaptureLatency() + result.getTargetingLatency();
        long measNanos = nowNanos - (long)(latencyMs * 1e6);
        Pose odomAtMeas = getOdomAtTime(measNanos);

        // --- Innovation Check (Outlier Rejection) ---
        // Convert Limelight pose (meters, degrees) to our system (inches, radians)
        double visionX = botpose.getPosition().x * M_TO_IN;
        double visionY = botpose.getPosition().y * M_TO_IN;
        // The robot's heading (yaw) is extracted from the Orientation of the Pose3D object.
        double visionHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        Pose visionAtMeas = new Pose(visionX, visionY, visionHeading);

        double dx = visionAtMeas.getX() - odomAtMeas.getX();
        double dy = visionAtMeas.getY() - odomAtMeas.getY();
        double dPos = Math.hypot(dx, dy);
        double dHeadDeg = Math.toDegrees(angleWrap(visionAtMeas.getHeading() - odomAtMeas.getHeading()));

        double speed = follower.getVelocity().getMagnitude();
        double linGate = (speed > minSpeedForRejectInPerS) ? linearGateIn : (linearGateIn * 1.5);
        double angGate = (speed > minSpeedForRejectInPerS) ? angularGateDeg : (angularGateDeg * 1.5);

        if (dPos > linGate || Math.abs(dHeadDeg) > angGate) {
            return; // Reject as outlier
        }

        // --- Fuse Poses ---
        double K = qualityWeight(result);
        if (K <= 0) return;

        Pose odomNow = follower.getPose();
        Pose visionNow = extrapolateToNow(visionAtMeas, odomAtMeas, odomNow);

        Pose fused = new Pose(
                odomNow.getX() + K * (visionNow.getX() - odomNow.getX()),
                odomNow.getY() + K * (visionNow.getY() - odomNow.getY()),
                angleWrap(odomNow.getHeading() + K * angleWrap(visionNow.getHeading() - odomNow.getHeading()))
        );

        follower.setPose(fused);
    }

    /**
     * Estimates the current vision pose by applying the odometry's movement since the
     * vision measurement was taken.
     */
    private Pose extrapolateToNow(Pose visionAtMeas, Pose odomAtMeas, Pose odomNow) {
        double dx = odomNow.getX() - odomAtMeas.getX();
        double dy = odomNow.getY() - odomAtMeas.getY();
        double dHeading = angleWrap(odomNow.getHeading() - odomAtMeas.getHeading());

        return new Pose(
                visionAtMeas.getX() + dx,
                visionAtMeas.getY() + dy,
                angleWrap(visionAtMeas.getHeading() + dHeading)
        );
    }

    /**
     * Wraps an angle to the range [-PI, PI].
     */
    private double angleWrap(double a) {
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }

    /**
     * Calculates a confidence weight (0.0 to 0.5) for a given Limelight measurement based on the number of tags seen.
     * A higher weight means the measurement is considered more trustworthy.
     */
    private double qualityWeight(LLResult result) {
        int numTags = result.getBotposeTagCount();
        // If we see no tags, the pose is invalid. Return a weight of 0.
        if (numTags < 1) {
            return 0.0;
        }

        // Start with a base weight for seeing at least one tag.
        // A single tag pose is usable, but less reliable than a multi-tag pose.
        double w = 0.15;

        // Increase the weight if we see more tags, as this increases confidence.
        if (numTags >= 2) w += 0.2;

        // Clamp to a maximum weight to prevent single measurements from drastically changing the pose
        return Math.min(0.5, w);
    }
}
