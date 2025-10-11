package org.firstinspires.ftc.teamcode.localisation;

import static org.firstinspires.ftc.teamcode.localisation.Constants.GATE_POS_M;
import static org.firstinspires.ftc.teamcode.localisation.Constants.GATE_TH_RAD;
import static org.firstinspires.ftc.teamcode.localisation.Constants.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.localisation.Constants.kPos;
import static org.firstinspires.ftc.teamcode.localisation.Constants.kTheta;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizerLimelight;

public class StateEstimator {
    OpMode opmode;

    // Fallback mode
    private boolean fallbackMode = false;

    // Pinpoint
    private final GoBildaPinpointDriver pinpoint;

    // April tag localizer
    AprilTagLocalizerLimelight aprilTagLocalizer;

    // Limelight telemetry init
    private boolean visionValid = false, visionAccepted = false;
    private double llX_last = Double.NaN, llY_last = Double.NaN, llTheta_last = Double.NaN;
    private double residX = 0, residY = 0, residTheta = 0;
    private double appliedX = 0, appliedY = 0, appliedTheta = 0; // gain * residual
    private long lastVisionAcceptMs = -1;
    private int framesWithVision = 0, framesAccepted = 0;
    private Pose2D lastFieldPose = new Pose2D(DistanceUnit.METER,0,0,AngleUnit.RADIANS,0);


    // Constructor
    public StateEstimator(OpMode opmode, GoBildaPinpointDriver pinpoint, AprilTagLocalizerLimelight aprilTagLocalizer) {
        this.opmode = opmode;
        this.pinpoint = pinpoint;
        this.aprilTagLocalizer = aprilTagLocalizer;
    }

    // Fallback
    public void toggleFallbackMode() {
        this.fallbackMode = !this.fallbackMode;
    }
    public void setFallbackMode(boolean enabled) {
        this.fallbackMode = enabled;
    }
    public boolean isFallbackMode() {
        return this.fallbackMode;
    }

    // Must call once per loop for updated data
    public void update() {
        pinpoint.update();
        if (!fallbackMode) {
            aprilTagLocalizer.update(pinpoint.getHeading(AngleUnit.DEGREES));
            lastFieldPose = getFieldPose();
        }
        if (TELEMETRY_ENABLED) addTelemetry();
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
        if (fallbackMode) {
            opmode.telemetry.addLine("Fallback Mode enabled for getFieldPose()");
            return getPose();
        }
        Pose2D est = getPose();

        LLResult result = aprilTagLocalizer.getResult();
        Pose3D mt2 = (result != null && result.isValid()) ? result.getBotpose_MT2() : null;

        visionValid = (mt2 != null);
        if (visionValid) {
            double llX = mt2.getPosition().x;
            double llY = mt2.getPosition().y;
            double llTheta = mt2.getOrientation().getYaw(AngleUnit.RADIANS);

            llX_last = llX; llY_last = llY; llTheta_last = llTheta;
            framesWithVision++;

            double dx = llX - est.getX(DistanceUnit.METER);
            double dy = llY - est.getY(DistanceUnit.METER);
            double dTheta = angleWrap(llTheta - est.getHeading(AngleUnit.RADIANS));

            residX = dx; residY = dy; residTheta = dTheta;

            boolean gate = Math.hypot(dx, dy) < GATE_POS_M && Math.abs(dTheta) < GATE_TH_RAD;
            if (gate) {
                appliedX = kPos * dx;
                appliedY = kPos * dy;
                appliedTheta = kTheta * dTheta;

                visionAccepted = true;
                framesAccepted++;
                lastVisionAcceptMs = System.currentTimeMillis();

                est = new Pose2D(
                        DistanceUnit.METER,
                        est.getX(DistanceUnit.METER) + appliedX,
                        est.getY(DistanceUnit.METER) + appliedY,
                        AngleUnit.RADIANS,
                        angleWrap(est.getHeading(AngleUnit.RADIANS) + appliedTheta)
                );
            } else {
                visionAccepted = false;
                appliedX = appliedY = appliedTheta = 0;
            }
        } else {
            visionAccepted = false;
            appliedX = appliedY = appliedTheta = 0;
        }
        return est;
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
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
            Pose2D est = lastFieldPose;
            double ex = est.getX(DistanceUnit.METER);
            double ey = est.getY(DistanceUnit.METER);
            double ehRad = est.getHeading(AngleUnit.RADIANS);
            double ehDeg = Math.toDegrees(ehRad);

            opmode.telemetry.addData("Pose (field)",
                    "(%.3f, %.3f) m  |  %.1f°", ex, ey, ehDeg);

            opmode.telemetry.addData("LL valid / accepted", "%b / %b", visionValid, visionAccepted);
            opmode.telemetry.addData("LL pose (field)",
                    "(%.3f, %.3f) m | %.1f°", llX_last, llY_last, Math.toDegrees(llTheta_last));

            double posErr = Math.hypot(residX, residY);
            opmode.telemetry.addData("LL residual",
                    "dx=%.3f dy=%.3f dθ=%.1f° | ‖pos‖=%.3f m",
                    residX, residY, Math.toDegrees(residTheta), posErr);

            opmode.telemetry.addData("Gate thresh", "< %.2f m  &  < %.1f°",
                    GATE_POS_M, Math.toDegrees(GATE_TH_RAD));

            opmode.telemetry.addData("Applied",
                    "x=%.3f  y=%.3f  θ=%.1f°",
                    appliedX, appliedY, Math.toDegrees(appliedTheta));

            long sinceMs = lastVisionAcceptMs < 0 ? -1 : (System.currentTimeMillis() - lastVisionAcceptMs);
            opmode.telemetry.addData("Since last accept [ms] / counts",
                    "%d  |  seen=%d  accepted=%d", sinceMs, framesWithVision, framesAccepted);

            opmode.telemetry.addData("Yaw to LL [deg]", "%.1f", Math.toDegrees(getHeading()));
        }
    }

    private static double angleWrap(double a) {
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
}