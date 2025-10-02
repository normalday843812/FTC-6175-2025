package org.firstinspires.ftc.teamcode.localisation;

import static org.firstinspires.ftc.teamcode.localisation.Constants.OUTLIER_POS_M;
import static org.firstinspires.ftc.teamcode.localisation.Constants.TELEMETRY_ENABLED;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class StateEstimator {
    OpMode opmode;

    // Fallback mode
    private boolean fallbackMode = false;

    // Pinpoint
    private final GoBildaPinpointDriver pinpoint;

    // April tag localizer
    AprilTagLocalizer aprilTagLocalizer;

    // EKF
    private final EkfPose2D ekf = new EkfPose2D();
    private int tagAccepted = 0, tagRejected = 0;

    // Constructor
    public StateEstimator(OpMode opmode, GoBildaPinpointDriver pinpoint, AprilTagLocalizer aprilTagLocalizer) {
        this.opmode = opmode;
        this.pinpoint = pinpoint;
        this.aprilTagLocalizer = aprilTagLocalizer;
        ekf.resetTo(0,0,0);
    }

    // Fallback
    // Toggle fallBackMode
    public void toggleFallbackMode() {
        this.fallbackMode = !this.fallbackMode;
    }
    // Set fallBackMode
    public void setFallbackMode(boolean enabled) {
        this.fallbackMode = enabled;
    }
    // Get fallbackMode
    public boolean isFallbackMode() {
        return this.fallbackMode;
    }

    // Must call once per loop for updated data
    public void update() {
        // Get odometry from pinpoint
        pinpoint.update();

        // If fallback is enabled, don't do any EKF or april tag stuff below this
        if (fallbackMode) {
            if (TELEMETRY_ENABLED) { addTelemetry(); }
            return;
        }

        // Predict from robot-frame velocities
        ChassisSpeeds vr = getChassisSpeedsRobot();
        ekf.predict(vr.vxMetersPerSecond, vr.vyMetersPerSecond, vr.omegaRadiansPerSecond, System.nanoTime());

        // Take detections and fuse them
        if (aprilTagLocalizer != null && aprilTagLocalizer.getAprilTag() != null) {
            List<AprilTagDetection> detections = aprilTagLocalizer.getAprilTag().getDetections();
            if (detections != null && !detections.isEmpty()) {
                Pose2D est = ekf.toPose2D();
                double ex = est.getX(DistanceUnit.METER), ey = est.getY(DistanceUnit.METER);
                for (AprilTagDetection d : detections) {
                    if (d == null || d.robotPose == null) continue;
                    if (d.metadata != null && d.metadata.name != null && d.metadata.name.contains("Obelisk")) continue;

                    if ( TELEMETRY_ENABLED ) {
                        opmode.telemetry.addLine("--- APRILTAGS ---");
                        aprilTagLocalizer.addTelemetry(d);
                    }

                    Pose3D rp = d.robotPose;
                    Position pMeters = rp.getPosition().toUnit(DistanceUnit.METER);
                    double zx = pMeters.x;
                    double zy = pMeters.y;
                    double zh = rp.getOrientation().getYaw(AngleUnit.RADIANS);

                    if (Math.hypot(zx - ex, zy - ey) > OUTLIER_POS_M) { tagRejected++; continue; }

                    boolean goodAprilTag = ekf.updateFromAprilTag(zx, zy, zh, d.decisionMargin);
                    if (goodAprilTag) {
                        tagAccepted++;
                        est = ekf.toPose2D();
                        ex = est.getX(DistanceUnit.METER); ey = est.getY(DistanceUnit.METER);
                    } else {
                        tagRejected++;
                    }
                }
            }
        }

        if (TELEMETRY_ENABLED) { addTelemetry(); }
    }

    // Returns odometry-frame pose (relative to last reset)
    public Pose2D getPose() {
        double x = pinpoint.getPosX(DistanceUnit.METER);
        double y = pinpoint.getPosY(DistanceUnit.METER);
        double h = pinpoint.getHeading(AngleUnit.RADIANS);
        return new Pose2D(DistanceUnit.METER, x, y, AngleUnit.RADIANS, h);
    }

    public double getHeading() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public ChassisSpeeds getChassisSpeedsField() {
        double vx = pinpoint.getVelX(DistanceUnit.METER);
        double vy = pinpoint.getVelY(DistanceUnit.METER);
        double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        return new ChassisSpeeds(vx, vy, omega);
    }

    public ChassisSpeeds getChassisSpeedsRobot() {
        double vxF = pinpoint.getVelX(DistanceUnit.METER);
        double vyF = pinpoint.getVelY(DistanceUnit.METER);
        double h = pinpoint.getHeading(AngleUnit.RADIANS);
        double cos = Math.cos(h), sin = Math.sin(h);

        double vxR = vxF * cos + vyF * sin;
        double vyR = -vxF * sin + vyF * cos;
        double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        return new ChassisSpeeds(vxR, vyR, omega);
    }

    public Pose2D getFieldPose() {
        if (fallbackMode) return getPose();
        return ekf.toPose2D();
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
        ekf.resetTo(0, 0, 0);
        tagAccepted = tagRejected = 0;
    }

    public void addTelemetry() {
        // Odometry-frame pose (Pinpoint)
        Pose2D odo = getPose();
        double ox = odo.getX(DistanceUnit.METER);
        double oy = odo.getY(DistanceUnit.METER);
        double ohRad = odo.getHeading(AngleUnit.RADIANS);

        // Velocities
        ChassisSpeeds vf = getChassisSpeedsField();
        ChassisSpeeds vr = getChassisSpeedsRobot();

        // Headings in degrees for readability
        double ohDeg = Math.toDegrees(ohRad);

        // Angular rates in deg/s
        double omegaFieldDeg = Math.toDegrees(vf.omegaRadiansPerSecond);
        double omegaRobotDeg = Math.toDegrees(vr.omegaRadiansPerSecond);

        opmode.telemetry.addLine(fallbackMode ? "--- StateEstimator Fallback ---" : "--- StateEstimator ---");

        opmode.telemetry.addData("Pinpoint pose (odom)",
                "(%.3f, %.3f) m  |  %.1f°",
                ox, oy, ohDeg);

        opmode.telemetry.addData("v_field [m/s, deg/s]",
                "vx=%.3f  vy=%.3f  ω=%.1f",
                vf.vxMetersPerSecond, vf.vyMetersPerSecond, omegaFieldDeg);

        opmode.telemetry.addData("v_robot [m/s, deg/s]",
                "vx=%.3f  vy=%.3f  ω=%.1f",
                vr.vxMetersPerSecond, vr.vyMetersPerSecond, omegaRobotDeg);

        if (!fallbackMode) {
            // EKF
            Pose2D est = getFieldPose();
            double ex = est.getX(DistanceUnit.METER);
            double ey = est.getY(DistanceUnit.METER);
            double ehRad = est.getHeading(AngleUnit.RADIANS);
            double ehDeg = Math.toDegrees(ehRad);

            opmode.telemetry.addData("EKF pose (field)",
                    "(%.3f, %.3f) m  |  %.1f°",
                    ex, ey, ehDeg);

            // AprilTags
            opmode.telemetry.addData("AprilTags (StateEstimator)",
                    "accepted=%d  rejected=%d  gate=%.2f m",
                    tagAccepted, tagRejected, OUTLIER_POS_M);
        }
    }

//    public int getTagAccepted() { return tagAccepted; }
//    public int getTagRejected() { return tagRejected; }
}