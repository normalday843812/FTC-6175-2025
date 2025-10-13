package org.firstinspires.ftc.teamcode.localisation;

import static org.firstinspires.ftc.teamcode.localisation.LocalisationConstants.*;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;

public class StateEstimator implements Localizer {
    // Hardware
    private final GoBildaPinpointDriver pinpoint;
    private final AprilTagLocalizer aprilTagLocalizer;

    // Telemetry
    private final TelemetryHelper tele;

    // Filters
    private final KalmanFilter kx;
    private final KalmanFilter ky;
    private final KalmanFilter kth;

    // State
    private Pose fusedIn = new Pose(0, 0, 0);
    private Pose velIn = new Pose(0, 0, 0);

    // Accumulators
    private double totalHeadingRad = 0.0;
    private double lastImuHeadingRad = 0.0;
    private boolean seeded = false;
    private boolean visionEnabled = true;

    // Vision bookkeeping
    private static final class VisionState {
        boolean valid = false;
        boolean accepted = false;
        long lastAcceptMs = -1;
        int framesWithVision = 0;
        int framesAccepted = 0;
        int lastTagCount = 0;
        double lastSpan = 0;
        double lastAvgDist = 0;
        long lastAgeMs = -1;
    }

    private static final class VisionPoseState {
        double llX = Double.NaN;
        double llY = Double.NaN;
        double llTheta = Double.NaN;
        Pose2D lastFieldPose = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.RADIANS, 0);
    }

    private static final class ResidualState {
        double x_m = 0;
        double y_m = 0;
        double theta_rad = 0;
        double gatePos_m = 0;
        double gateYaw_rad = 0;
    }

    private final VisionState vision = new VisionState();
    private final VisionPoseState visionPose = new VisionPoseState();
    private final ResidualState residual = new ResidualState();

    // Filter params tied to gate floors
    private static KalmanFilterParameters paramsPos() {
        double sigma = GATE_POS_MIN_M / 3.0;
        double modelVar = (0.25 * sigma) * (0.25 * sigma);
        double dataVar = sigma * sigma;
        return new KalmanFilterParameters(modelVar, dataVar);
    }

    private static KalmanFilterParameters paramsYaw() {
        double sigmaYaw = GATE_YAW_MIN_RAD / 3.0;
        double modelVar = (0.25 * sigmaYaw) * (0.25 * sigmaYaw);
        double dataVar = sigmaYaw * sigmaYaw;
        return new KalmanFilterParameters(modelVar, dataVar);
    }

    public StateEstimator(OpMode opmode, GoBildaPinpointDriver pinpoint, AprilTagLocalizer aprilTagLocalizer) {
        this.pinpoint = pinpoint;
        this.aprilTagLocalizer = aprilTagLocalizer;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);

        this.kx = new KalmanFilter(paramsPos());
        this.ky = new KalmanFilter(paramsPos());
        this.kth = new KalmanFilter(paramsYaw());
    }

    public void seedFieldPose(double x_m, double y_m, double hRad) {
        setStartPose(new Pose(x_m * M_TO_IN, y_m * M_TO_IN, hRad));
    }

    public void setVisionEnabled(boolean enabled) {
        this.visionEnabled = enabled;
    }

    public boolean isVisionEnabled() {
        return visionEnabled;
    }

    @Override
    public void update() {
        // Odometry (meters)
        pinpoint.update();
        double xM = pinpoint.getPosX(DistanceUnit.METER);
        double yM = pinpoint.getPosY(DistanceUnit.METER);
        double h = pinpoint.getHeading(AngleUnit.RADIANS);

        double vxMps = pinpoint.getVelX(DistanceUnit.METER);
        double vyMps = pinpoint.getVelY(DistanceUnit.METER);
        double wRps = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        // Pedro boundary conversion
        double xInOdo = xM * M_TO_IN;
        double yInOdo = yM * M_TO_IN;
        double vxIn = vxMps * M_TO_IN;
        double vyIn = vyMps * M_TO_IN;

        if (!seeded) {
            fusedIn = new Pose(xInOdo, yInOdo, h);
            lastImuHeadingRad = h;
            seeded = true;
        }

        // Unbounded heading accumulator
        double dH = wrap(h - lastImuHeadingRad);
        totalHeadingRad += dH;
        lastImuHeadingRad = h;

        // Let Limelight know our current field-referenced yaw
        aprilTagLocalizer.update(Math.toDegrees(h));

        // Vision
        LLResult r = aprilTagLocalizer.getResult();
        vision.lastAgeMs = aprilTagLocalizer.getMillisSinceLastUpdate();
        boolean fresh = r != null && r.isValid() && vision.lastAgeMs <= LL_STALE_MS;

        boolean accepted = false;
        double mxIn = 0, myIn = 0, mh = 0;

        if (visionEnabled && fresh) {
            Pose3D p = r.getBotpose_MT2();
            if (!isZeroPose(p)) {
                vision.lastTagCount = r.getBotposeTagCount();
                vision.lastSpan = r.getBotposeSpan();
                vision.lastAvgDist = r.getBotposeAvgDist();

                visionPose.llX = p.getPosition().x;
                visionPose.llY = p.getPosition().y;
                visionPose.llTheta = p.getOrientation().getYaw(AngleUnit.RADIANS);

                double[] std = r.getStddevMt2();
                double sx = (std != null && std.length >= 2 && std[0] > 0) ? std[0] : GATE_POS_MAX_M / (3.0 * Math.sqrt(2.0));
                double sy = (std != null && std.length >= 2 && std[1] > 0) ? std[1] : GATE_POS_MAX_M / (3.0 * Math.sqrt(2.0));
                double syaw = (std != null && std.length >= 6 && std[5] > 0) ? Math.abs(std[5]) : GATE_YAW_MAX_RAD / 3.0;

                double sigmaPos = Math.hypot(sx, sy);
                boolean multi = vision.lastTagCount >= 2;
                boolean singleOk = (!multi) && (3.0 * sigmaPos <= SINGLE_TAG_3SIGMA_POS_MAX_M);
                boolean areaOk = r.getBotposeAvgArea() >= MIN_AVG_AREA;
                vision.valid = areaOk && (multi || singleOk);

                residual.x_m = visionPose.llX - xM;
                residual.y_m = visionPose.llY - yM;
                residual.theta_rad = wrap(visionPose.llTheta - h);

                Gates g = gatesFrom(sx, sy, syaw);
                residual.gatePos_m = g.gatePos_m;
                residual.gateYaw_rad = g.gateYaw_rad;

                boolean pass = Math.hypot(residual.x_m, residual.y_m) < g.gatePos_m
                        && Math.abs(residual.theta_rad) < g.gateYaw_rad;

                if (vision.valid && pass) {
                    mxIn = visionPose.llX * M_TO_IN;
                    myIn = visionPose.llY * M_TO_IN;
                    mh = visionPose.llTheta;
                    accepted = true;
                }
            } else {
                vision.valid = false;
            }
        } else {
            vision.valid = false;
        }

        // Fusion
        if (accepted) {
            double dxIn = xInOdo - fusedIn.getX();
            double dyIn = yInOdo - fusedIn.getY();
            double dHIn = wrap(h - fusedIn.getHeading());

            kx.update(dxIn, mxIn);
            ky.update(dyIn, myIn);
            kth.update(dHIn, mh);

            fusedIn = new Pose(
                    fusedIn.getX() + kx.getState(),
                    fusedIn.getY() + ky.getState(),
                    wrap(fusedIn.getHeading() + kth.getState())
            );

            vision.accepted = true;
            vision.framesAccepted++;
            vision.lastAcceptMs = System.currentTimeMillis();
        } else {
            fusedIn = new Pose(xInOdo, yInOdo, h);
            vision.accepted = false;
        }
        if (vision.valid) vision.framesWithVision++;

        velIn = new Pose(vxIn, vyIn, wRps);

        // Legacy field pose in meters for your other code
        visionPose.lastFieldPose = new Pose2D(
                DistanceUnit.METER,
                fusedIn.getX() * IN_TO_M,
                fusedIn.getY() * IN_TO_M,
                AngleUnit.RADIANS,
                fusedIn.getHeading()
        );

        addTelemetry(xM, yM, h, r);
    }

    @Override
    public Pose getPose() {
        return fusedIn;
    }

    @Override
    public Pose getVelocity() {
        return velIn;
    }

    @Override
    public Vector getVelocityVector() {
        return new Vector(velIn.getX(), velIn.getY());
    }

    @Override
    public void setStartPose(Pose p) {
        double xM = p.getX() * IN_TO_M;
        double yM = p.getY() * IN_TO_M;
        double h = p.getHeading();
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.METER, xM, yM, AngleUnit.RADIANS, h));
        pinpoint.setHeading(h, AngleUnit.RADIANS);
        fusedIn = p.copy();
        lastImuHeadingRad = h;
        totalHeadingRad = 0.0;
        seeded = true;
    }

    @Override
    public void setPose(Pose p) {
        setStartPose(p);
    }

    @Override
    public double getTotalHeading() {
        return totalHeadingRad;
    }

    @Override
    public double getForwardMultiplier() {
        return 1.0;
    }

    @Override
    public double getLateralMultiplier() {
        return 1.0;
    }

    @Override
    public double getTurningMultiplier() {
        return 1.0;
    }

    @Override
    public void resetIMU() {
        pinpoint.resetPosAndIMU();
    }

    @Override
    public double getIMUHeading() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(fusedIn.getX()) || Double.isNaN(fusedIn.getY()) || Double.isNaN(fusedIn.getHeading());
    }

    public Pose2D getFieldPose() {
        return visionPose.lastFieldPose;
    }

    public double getFusedHeading(AngleUnit unit) {
        return unit == AngleUnit.DEGREES ? Math.toDegrees(fusedIn.getHeading()) : fusedIn.getHeading();
    }

    private static boolean isZeroPose(Pose3D p) {
        if (p == null) return true;
        double x = p.getPosition().x, y = p.getPosition().y, z = p.getPosition().z;
        double yaw = p.getOrientation().getYaw(AngleUnit.RADIANS);
        return Math.abs(x) < ZERO_EPS_POS_M && Math.abs(y) < ZERO_EPS_POS_M &&
                Math.abs(z) < ZERO_EPS_POS_M && Math.abs(yaw) < ZERO_EPS_YAW_RAD;
    }

    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static final class Gates {
        double gatePos_m, gateYaw_rad;
    }

    private static Gates gatesFrom(double sx_m, double sy_m, double syaw_rad) {
        double sigmaPos = Math.hypot(sx_m, sy_m);
        double sigmaYaw = Math.abs(syaw_rad);
        Gates g = new Gates();
        g.gatePos_m = clamp(3.0 * sigmaPos, GATE_POS_MIN_M, GATE_POS_MAX_M);
        g.gateYaw_rad = clamp(3.0 * sigmaYaw, GATE_YAW_MIN_RAD, GATE_YAW_MAX_RAD);
        return g;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void addTelemetry(double xM, double yM, double h, LLResult r) {
        tele.addLine("--- StateEstimator [Pedro Localizer] ---")
                .addData("Pose_fused [in | deg]", "(%.2f, %.2f) | %.1f°",
                        fusedIn.getX(), fusedIn.getY(), Math.toDegrees(fusedIn.getHeading()))
                .addData("Pose_odom [m | deg]", "(%.3f, %.3f) | %.1f°",
                        xM, yM, Math.toDegrees(h))
                .addData("Vel [in/s | deg/s]", "vx=%.2f vy=%.2f ω=%.1f",
                        velIn.getX(), velIn.getY(), Math.toDegrees(velIn.getHeading()))
                .addData("LL fresh/valid/accepted", "%b / %b / %b",
                        (r != null && r.isValid()), vision.valid, vision.accepted)
                .addData("LL age[ms] tags/span/avgDist", "%d | %d / %.2f / %.2f",
                        vision.lastAgeMs, vision.lastTagCount, vision.lastSpan, vision.lastAvgDist)
                .addData("LL pose [m | deg]", "(%.3f, %.3f) | %.1f°",
                        visionPose.llX, visionPose.llY, Math.toDegrees(visionPose.llTheta))
                .addData("Residuals [m | deg]", "dx=%.3f dy=%.3f dθ=%.2f°",
                        residual.x_m, residual.y_m, Math.toDegrees(residual.theta_rad))
                .addData("Gates [m | deg]", "pos<%.2f yaw<%.1f°",
                        residual.gatePos_m, Math.toDegrees(residual.gateYaw_rad))
                .addData("Frames seen/accepted", "%d / %d", vision.framesWithVision, vision.framesAccepted);
    }
}
