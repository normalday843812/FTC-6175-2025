package org.firstinspires.ftc.teamcode.localisation;

import static org.firstinspires.ftc.teamcode.localisation.Constants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.localisation.Constants.OUTLIER_POS_M;
import static org.firstinspires.ftc.teamcode.localisation.Constants.PARALLEL_Y_M;
import static org.firstinspires.ftc.teamcode.localisation.Constants.PERPENDICULAR_X_M;
import static org.firstinspires.ftc.teamcode.localisation.Constants.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.localisation.Constants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.localisation.Constants.WHEEL_RADIUS_M;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class StateEstimatorTwoWheel {
    private final OpMode opmode;
    // Sensors
    private final DcMotorEx encParallel;
    private final DcMotorEx encPerp;
    private final IMU imu;

    // AprilTags
    private final AprilTagLocalizer aprilTagLocalizer;

    // EKF for field pose
    private final EkfPose2D ekf = new EkfPose2D();
    private int tagAccepted = 0, tagRejected = 0;

    // Internal state for odometry & velocities
    private long lastNanos = -1;
    private double yawOffset;
    private double lastHeading;

    private int lastTicksPar;
    private int lastTicksPerp;

    private double xOdo = 0.0, yOdo = 0.0; // odometry-frame pose since last reset

    private ChassisSpeeds lastVr = new ChassisSpeeds(0,0,0); // robot frame
    private ChassisSpeeds lastVf = new ChassisSpeeds(0,0,0); // field frame

    // Constructor
    public StateEstimatorTwoWheel(OpMode opmode,
                                  DcMotorEx encParallel,
                                  DcMotorEx encPerp,
                                  IMU imu,
                                  AprilTagLocalizer aprilTagLocalizer) {
        this.opmode = opmode;
        this.encParallel = encParallel;
        this.encPerp = encPerp;
        this.imu = imu;
        this.aprilTagLocalizer = aprilTagLocalizer;

        // Ensure odometry encoders are zeroed and running freely
        try {
            this.encParallel.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.encPerp.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } catch (Exception ignored) {}
        this.encParallel.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.encPerp.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize EKF field pose to (0,0,0)
        ekf.resetTo(0,0,0);

        yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        lastHeading = 0.0;

        lastTicksPar = encParallel.getCurrentPosition();
        lastTicksPerp = encPerp.getCurrentPosition();
    }

    // Must call once per loop for updated data
    public void update() {
        long now = System.nanoTime();
        double dt = (lastNanos < 0) ? 0.0 : (now - lastNanos) * 1e-9;
        lastNanos = now;
        if (dt <= 0) {
            if (TELEMETRY_ENABLED) addTelemetry();
            return;
        }

        // Read sensors
        int ticksPar = encParallel.getCurrentPosition();
        int ticksPerp = encPerp.getCurrentPosition();

        int dTicksPar = ticksPar - lastTicksPar;
        int dTicksPerp = ticksPerp - lastTicksPerp;
        lastTicksPar = ticksPar;
        lastTicksPerp = ticksPerp;

        double distPar = ticksToMeters(dTicksPar);
        double distPerp = ticksToMeters(dTicksPerp);

        double headingRaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double heading = wrapRad(headingRaw - yawOffset);
        double dH = wrapRad(heading - lastHeading);

        // Remove rotation-induced components to get true robot-frame translation
        double dxR = distPar + PARALLEL_Y_M * dH;
        double dyR = distPerp - PERPENDICULAR_X_M * dH;

        // Robot-frame velocities
        double vxR = dxR / dt;
        double vyR = dyR / dt;

        // Heading rate from IMU
        double omegaDeg = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
        double omega = Math.toRadians(omegaDeg); // rad/s

        lastVr = new ChassisSpeeds(vxR, vyR, omega);

        // Field-frame velocities (rotate by heading)
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double vxF = vxR * cos - vyR * sin;
        double vyF = vxR * sin + vyR * cos;
        lastVf = new ChassisSpeeds(vxF, vyF, omega);

        // Integrate odometry-frame pose
        xOdo += vxF * dt;
        yOdo += vyF * dt;
        lastHeading = heading;

        // EKF predict with robot-frame velocities
        ekf.predict(vxR, vyR, omega, now);

        // Fuse AprilTags (same logic as before)
        if (aprilTagLocalizer != null && aprilTagLocalizer.getAprilTag() != null) {
            List<AprilTagDetection> detections = aprilTagLocalizer.getAprilTag().getDetections();
            if (detections != null && !detections.isEmpty()) {
                Pose2D est = ekf.toPose2D();
                double ex = est.getX(DistanceUnit.METER), ey = est.getY(DistanceUnit.METER);

                for (AprilTagDetection d : detections) {
                    if (d == null || d.robotPose == null) continue;
                    if (d.metadata != null && d.metadata.name != null && d.metadata.name.contains("Obelisk")) continue;

                    if (TELEMETRY_ENABLED) {
                        opmode.telemetry.addLine("--- APRILTAGS ---");
                        aprilTagLocalizer.addTelemetry(d);
                    }

                    Pose3D rp = d.robotPose;
                    Position pMeters = rp.getPosition().toUnit(DistanceUnit.METER);
                    double zx = pMeters.x;
                    double zy = pMeters.y;
                    double zh = rp.getOrientation().getYaw(AngleUnit.RADIANS);

                    if (Math.hypot(zx - ex, zy - ey) > OUTLIER_POS_M) { tagRejected++; continue; }

                    boolean good = ekf.updateFromAprilTag(zx, zy, zh, d.decisionMargin);
                    if (good) {
                        tagAccepted++;
                        est = ekf.toPose2D();
                        ex = est.getX(DistanceUnit.METER);
                        ey = est.getY(DistanceUnit.METER);
                    } else {
                        tagRejected++;
                    }
                }
            }
        }

        if (TELEMETRY_ENABLED) addTelemetry();
    }

    // Odometry-frame pose (relative to last reset)
    public Pose2D getPose() {
        return new Pose2D(DistanceUnit.METER, xOdo, yOdo, AngleUnit.RADIANS, lastHeading);
    }

    public double getHeading() { return lastHeading; }

    public ChassisSpeeds getChassisSpeedsField() { return lastVf; }

    public ChassisSpeeds getChassisSpeedsRobot() { return lastVr; }

    public Pose2D getFieldPose() { return ekf.toPose2D(); }

    public void reset() {
        // Zero encoders
        try {
            encParallel.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encPerp.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } catch (Exception ignored) {}
        encParallel.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encPerp.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Zero heading
        yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        lastHeading = 0.0;

        // Zero odometry integration and counters
        xOdo = 0.0; yOdo = 0.0;
        lastTicksPar = encParallel.getCurrentPosition();
        lastTicksPerp = encPerp.getCurrentPosition();
        lastNanos = -1;

        // Reset EKF
        ekf.resetTo(0, 0, 0);
        tagAccepted = tagRejected = 0;
    }

    public void addTelemetry() {
        Pose2D odo = getPose();
        double ox = odo.getX(DistanceUnit.METER), oy = odo.getY(DistanceUnit.METER);
        double ohDeg = Math.toDegrees(odo.getHeading(AngleUnit.RADIANS));

        Pose2D est = getFieldPose();
        double ex = est.getX(DistanceUnit.METER), ey = est.getY(DistanceUnit.METER);
        double ehDeg = Math.toDegrees(est.getHeading(AngleUnit.RADIANS));

        double omegaFieldDeg = Math.toDegrees(lastVf.omegaRadiansPerSecond);
        double omegaRobotDeg = Math.toDegrees(lastVr.omegaRadiansPerSecond);

        opmode.telemetry.addLine("--- StateEstimator (2-wheel + IMU) ---");
        opmode.telemetry.addData("Odo pose (odom)", "(%.3f, %.3f) m  |  %.1f°", ox, oy, ohDeg);
        opmode.telemetry.addData("EKF pose (field)", "(%.3f, %.3f) m  |  %.1f°", ex, ey, ehDeg);
        opmode.telemetry.addData("v_field [m/s, deg/s]", "vx=%.3f  vy=%.3f  ω=%.1f", lastVf.vxMetersPerSecond, lastVf.vyMetersPerSecond, omegaFieldDeg);
        opmode.telemetry.addData("v_robot [m/s, deg/s]", "vx=%.3f  vy=%.3f  ω=%.1f", lastVr.vxMetersPerSecond, lastVr.vyMetersPerSecond, omegaRobotDeg);
        opmode.telemetry.addData("AprilTags", "accepted=%d  rejected=%d  gate=%.2f m", tagAccepted, tagRejected, OUTLIER_POS_M);
    }

    // Helpers
    private static double ticksToMeters(int ticks) {
        double metersPerRev = 2.0 * Math.PI * WHEEL_RADIUS_M * GEAR_RATIO;
        return ticks * (metersPerRev / TICKS_PER_REV);
    }

    private static double wrapRad(double a) {
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
}