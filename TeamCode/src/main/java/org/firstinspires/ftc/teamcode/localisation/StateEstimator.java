//package org.firstinspires.ftc.teamcode.localisation;
//
//import static org.firstinspires.ftc.teamcode.config.GlobalConfig.FALLBACK_MODE;
//import static org.firstinspires.ftc.teamcode.config.GlobalConfig.IN_TO_M;
//import static org.firstinspires.ftc.teamcode.config.GlobalConfig.M_TO_IN;
//import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.GATE_POS_MAX_M;
//import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.GATE_POS_MIN_M;
//import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.GATE_YAW_MAX_RAD;
//import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.GATE_YAW_MIN_RAD;
//import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.LL_STALE_MS;
//import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.MIN_AVG_AREA;
//import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.TELEMETRY_ENABLED;
//import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.ZERO_EPS_POS_M;
//import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.ZERO_EPS_YAW_RAD;
//
//import android.annotation.SuppressLint;
//
//import com.pedropathing.control.KalmanFilter;
//import com.pedropathing.control.KalmanFilterParameters;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.localization.Localizer;
//import com.pedropathing.math.Vector;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
//import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;
//
//public class StateEstimator implements Localizer {
//    /* Would like to mention that using PedroPathing's KF is a bit of a black box, but it should
//    be better for integration purposes */
//    // Hardware
//    private final GoBildaPinpointDriver pinpoint;
//    private final AprilTagLocalizer aprilTagLocalizer;
//
//    // Telemetry
//    private final TelemetryHelper tele;
//
//    // Filters
//    private final KalmanFilter kx;
//    private final KalmanFilter ky;
//    private final KalmanFilter kth;
//
//    // State
//    private Pose fusedIn = new Pose(0, 0, 0);
//    private Pose velIn = new Pose(0, 0, 0);
//
//    // Accumulators
//    private double totalHeadingRad = 0.0;
//    private double lastImuHeadingRad = 0.0;
//    private boolean seeded = false;
//    private boolean visionEnabled = true;
//
//    private boolean frameAlignedToLL = false;
//
//    // Vision bookkeeping
//    private static final class VisionState {
//        boolean valid = false;
//        boolean accepted = false;
//        int framesWithVision = 0;
//        int framesAccepted = 0;
//        int lastTagCount = 0;
//        double lastSpan = 0;
//        double lastAvgDist = 0;
//        long lastAgeMs = -1;
//    }
//
//    private static final class VisionPoseState {
//        double llX = Double.NaN;
//        double llY = Double.NaN;
//        double llTheta = Double.NaN;
//        Pose2D lastFieldPose = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.RADIANS, 0);
//    }
//
//    private static final class ResidualState {
//        double x_m = 0;
//        double y_m = 0;
//        double theta_rad = 0;
//        double gatePos_m = 0;
//        double gateYaw_rad = 0;
//    }
//
//    private final VisionState vision = new VisionState();
//    private final VisionPoseState visionPose = new VisionPoseState();
//    private final ResidualState residual = new ResidualState();
//
//    // Filter params tied to gate floors
//    private static KalmanFilterParameters paramsPos() {
//        double sigma = GATE_POS_MIN_M / 3.0;
//        double modelVar = (0.25 * sigma) * (0.25 * sigma);
//        double dataVar = sigma * sigma;
//        return new KalmanFilterParameters(modelVar, dataVar);
//    }
//
//    private static KalmanFilterParameters paramsYaw() {
//        double sigmaYaw = GATE_YAW_MIN_RAD / 3.0;
//        double modelVar = (0.25 * sigmaYaw) * (0.25 * sigmaYaw);
//        double dataVar = sigmaYaw * sigmaYaw;
//        return new KalmanFilterParameters(modelVar, dataVar);
//    }
//
//    public StateEstimator(OpMode opmode, GoBildaPinpointDriver pinpoint, AprilTagLocalizer aprilTagLocalizer) {
//        this.pinpoint = pinpoint;
//        this.aprilTagLocalizer = aprilTagLocalizer;
//        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
//
//        this.kx = new KalmanFilter(paramsPos());
//        this.ky = new KalmanFilter(paramsPos());
//        this.kth = new KalmanFilter(paramsYaw());
//    }
//
//    public void seedFieldPose(double x_m, double y_m, double hRad) {
//        setStartPose(new Pose(x_m * M_TO_IN, y_m * M_TO_IN, hRad));
//    }
//
//    public void setVisionEnabled(boolean enabled) {
//        this.visionEnabled = enabled;
//    }
//
//    public boolean isVisionEnabled() {
//        return visionEnabled;
//    }
//
//    @Override
//    public void update() {
//        // Odometry (meters)
//        pinpoint.update();
//        double xM = pinpoint.getPosX(DistanceUnit.METER);
//        double yM = pinpoint.getPosY(DistanceUnit.METER);
//        double h  = pinpoint.getHeading(AngleUnit.RADIANS);
//
//        double vxMps = pinpoint.getVelX(DistanceUnit.METER);
//        double vyMps = pinpoint.getVelY(DistanceUnit.METER);
//        double wRps  = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
//
//        // Seed once
//        if (!seeded) {
//            fusedIn = new Pose(xM * M_TO_IN, yM * M_TO_IN, h);
//            lastImuHeadingRad = h;
//            totalHeadingRad = 0.0;
//            seeded = true;
//            // Initialize filters at the current pose
//            kx.reset(fusedIn.getX(), 1.0, 1.0);
//            ky.reset(fusedIn.getY(), 1.0, 1.0);
//            kth.reset(fusedIn.getHeading(), 1.0, 1.0);
//        } else {
//            double dH = wrap(h - lastImuHeadingRad);
//            totalHeadingRad += dH;
//            lastImuHeadingRad = h;
//        }
//
//        // Let Limelight know our current field-referenced yaw
//        aprilTagLocalizer.update(Math.toDegrees(h));
//
//        if (FALLBACK_MODE) {
//            fusedIn = new Pose(xM * M_TO_IN, yM * M_TO_IN, wrap(h));
//            double vxIn = vxMps * M_TO_IN, vyIn = vyMps * M_TO_IN;
//            double c = Math.cos(h), s = Math.sin(h);
//            velIn = new Pose(vxIn * c - vyIn * s, vxIn * s + vyIn * c, wRps);
//
//            vision.valid = false;
//            vision.accepted = false;
//            vision.lastAgeMs = -1;
//            addTelemetry(xM, yM, h, null);
//            return;
//        }
//
//        // Vision
//        LLResult r = aprilTagLocalizer.getResult();
//        vision.lastAgeMs = aprilTagLocalizer.getMillisSinceLastUpdate();
//        boolean fresh = r != null && r.isValid() && visionEnabled && (vision.lastAgeMs <= LL_STALE_MS);
//
//        int tagCount = 0;
//        Pose3D p;
//        double llx_m = Double.NaN, lly_m = Double.NaN, llh = Double.NaN;
//
//        if (fresh) {
//            tagCount = r.getBotposeTagCount();
//            p = r.getBotpose_MT2();
//            if (tagCount >= 1 && isFinitePose(p) && !isZeroPose(p)) {
//                llx_m = p.getPosition().x;
//                lly_m = p.getPosition().y;
//                llh   = p.getOrientation().getYaw(AngleUnit.RADIANS);
//            } else {
//                fresh = false;
//            }
//        }
//
//        // TODO: add better documentation
//        if (!frameAlignedToLL) {
//            if (fresh) {
//                double[] std = r.getStddevMt2();
//                double sx = (std != null && std.length >= 2 && isFinite(std[0])) ? std[0] : 0.25;
//                double sy = (std != null && std.length >= 2 && isFinite(std[1])) ? std[1] : 0.25;
//                double syaw = (std != null && std.length >= 6 && isFinite(std[5])) ? Math.abs(std[5]) : Math.toRadians(15);
//                double sigmaPos = Math.hypot(sx, sy);
//                boolean ok = sigmaPos <= 0.35 && syaw <= Math.toRadians(15);
//
//                vision.lastTagCount = tagCount;
//                vision.lastSpan = r.getBotposeSpan();
//                vision.lastAvgDist = r.getBotposeAvgDist();
//                boolean areaOk = r.getBotposeAvgArea() >= MIN_AVG_AREA;
//
//                if (ok && areaOk) {
//                    pinpoint.setPosition(new Pose2D(DistanceUnit.METER, llx_m, lly_m, AngleUnit.RADIANS, llh));
//
//                    // Set fused pose
//                    fusedIn = new Pose(llx_m * M_TO_IN, lly_m * M_TO_IN, wrap(llh));
//
//                    // Field-frame velocities
//                    double vxIn = vxMps * M_TO_IN, vyIn = vyMps * M_TO_IN;
//                    double c = Math.cos(llh), s = Math.sin(llh);
//                    velIn = new Pose(vxIn * c - vyIn * s, vxIn * s + vyIn * c, wRps);
//
//                    frameAlignedToLL = true;
//                    vision.valid = true;
//                    vision.accepted = true;
//                    vision.framesAccepted++;
//                    vision.framesWithVision++;
//                    vision.lastAgeMs = aprilTagLocalizer.getMillisSinceLastUpdate();
//
//                    // Reset KFs
//                    kx.reset(fusedIn.getX(), 1.0, 1.0);
//                    ky.reset(fusedIn.getY(), 1.0, 1.0);
//                    kth.reset(fusedIn.getHeading(), 1.0, 1.0);
//
//                    // Cache LL pose for telemetry
//                    visionPose.llX = llx_m;
//                    visionPose.llY = lly_m;
//                    visionPose.llTheta = llh;
//                    visionPose.lastFieldPose = new Pose2D(DistanceUnit.METER, llx_m, lly_m, AngleUnit.RADIANS, llh);
//
//                    residual.x_m = 0;
//                    residual.y_m = 0;
//                    residual.theta_rad = 0;
//                    residual.gatePos_m = GATE_POS_MIN_M;
//                    residual.gateYaw_rad = GATE_YAW_MIN_RAD;
//
//                    addTelemetry(llx_m, lly_m, llh, r);
//                    return;
//                }
//            }
//
//            fusedIn = new Pose(xM * M_TO_IN, yM * M_TO_IN, wrap(h));
//            double vxIn = vxMps * M_TO_IN, vyIn = vyMps * M_TO_IN;
//            double c = Math.cos(h), s = Math.sin(h);
//            velIn = new Pose(vxIn * c - vyIn * s, vxIn * s + vyIn * c, wRps);
//
//            vision.valid = fresh;
//            vision.accepted = false;
//            if (fresh) vision.framesWithVision++;
//
//            // Telemetry LL pose cache
//            if (fresh) {
//                visionPose.llX = llx_m;
//                visionPose.llY = lly_m;
//                visionPose.llTheta = llh;
//                visionPose.lastFieldPose = new Pose2D(DistanceUnit.METER, llx_m, lly_m, AngleUnit.RADIANS, llh);
//            } else {
//                visionPose.llX = Double.NaN;
//                visionPose.llY = Double.NaN;
//                visionPose.llTheta = Double.NaN;
//            }
//
//            // Residuals not meaningful pre-alignment
//            residual.x_m = 0;
//            residual.y_m = 0;
//            residual.theta_rad = 0;
//            residual.gatePos_m = 0;
//            residual.gateYaw_rad = 0;
//
//            addTelemetry(xM, yM, h, r);
//            return;
//        }
//
//        // If already aligned go to pinpoint only
//        fusedIn = new Pose(xM * M_TO_IN, yM * M_TO_IN, wrap(h));
//        double vxIn = vxMps * M_TO_IN, vyIn = vyMps * M_TO_IN;
//        double c = Math.cos(h), s = Math.sin(h);
//        velIn = new Pose(vxIn * c - vyIn * s, vxIn * s + vyIn * c, wRps);
//
//        // Report, but do not fuse
//        if (fresh) {
//            vision.valid = true;
//            vision.accepted = false;
//            vision.framesWithVision++;
//            visionPose.llX = llx_m;
//            visionPose.llY = lly_m;
//            visionPose.llTheta = llh;
//            visionPose.lastFieldPose = new Pose2D(DistanceUnit.METER, llx_m, lly_m, AngleUnit.RADIANS, llh);
//
//            residual.x_m = llx_m - xM;
//            residual.y_m = lly_m - yM;
//            residual.theta_rad = wrap(llh - h);
//
//            double[] std = r.getStddevMt2();
//            double sx = (std != null && std.length >= 2 && isFinite(std[0])) ? std[0] : GATE_POS_MIN_M;
//            double sy = (std != null && std.length >= 2 && isFinite(std[1])) ? std[1] : GATE_POS_MIN_M;
//            Gates g = gatesFrom(sx, sy, Math.abs(residual.theta_rad));
//            residual.gatePos_m = g.gatePos_m;
//            residual.gateYaw_rad = g.gateYaw_rad;
//
//            vision.lastTagCount = tagCount;
//            vision.lastSpan = r.getBotposeSpan();
//            vision.lastAvgDist = r.getBotposeAvgDist();
//        } else {
//            vision.valid = false;
//            vision.accepted = false;
//            visionPose.llX = Double.NaN;
//            visionPose.llY = Double.NaN;
//            visionPose.llTheta = Double.NaN;
//            residual.x_m = 0;
//            residual.y_m = 0;
//            residual.theta_rad = 0;
//            residual.gatePos_m = 0;
//            residual.gateYaw_rad = 0;
//        }
//
//        addTelemetry(xM, yM, h, r);
//    }
//
//    @Override
//    public Pose getPose() {
//        return fusedIn;
//    }
//
//    @Override
//    public Pose getVelocity() {
//        return velIn;
//    }
//
//    @Override
//    public Vector getVelocityVector() {
//        return new Vector(velIn.getX(), velIn.getY());
//    }
//
//    @Override
//    public void setStartPose(Pose p) {
//        double xM = p.getX() * IN_TO_M;
//        double yM = p.getY() * IN_TO_M;
//        double h = p.getHeading();
//        pinpoint.resetPosAndIMU();
//        pinpoint.setPosition(new Pose2D(DistanceUnit.METER, xM, yM, AngleUnit.RADIANS, h));
//        pinpoint.setHeading(h, AngleUnit.RADIANS);
//        fusedIn = p.copy();
//        lastImuHeadingRad = h;
//        totalHeadingRad = 0.0;
//        seeded = true;
//    }
//
//    @Override
//    public void setPose(Pose p) {
//        setStartPose(p);
//    }
//
//    @Override
//    public double getTotalHeading() {
//        return totalHeadingRad;
//    }
//
//    @Override
//    public double getForwardMultiplier() {
//        return 1.0;
//    }
//
//    @Override
//    public double getLateralMultiplier() {
//        return 1.0;
//    }
//
//    @Override
//    public double getTurningMultiplier() {
//        return 1.0;
//    }
//
//    @Override
//    public void resetIMU() {
//        pinpoint.resetPosAndIMU();
//    }
//
//    @Override
//    public double getIMUHeading() {
//        return pinpoint.getHeading(AngleUnit.RADIANS);
//    }
//
//    @Override
//    public boolean isNAN() {
//        return Double.isNaN(fusedIn.getX()) || Double.isNaN(fusedIn.getY()) || Double.isNaN(fusedIn.getHeading());
//    }
//
//    public Pose2D getFieldPose() {
//        return visionPose.lastFieldPose;
//    }
//
//    public double getFusedHeading(AngleUnit unit) {
//        return unit == AngleUnit.DEGREES ? Math.toDegrees(fusedIn.getHeading()) : fusedIn.getHeading();
//    }
//
//    private static boolean isZeroPose(Pose3D p) {
//        if (p == null) return true;
//        double x = p.getPosition().x, y = p.getPosition().y, z = p.getPosition().z;
//        double yaw = p.getOrientation().getYaw(AngleUnit.RADIANS);
//        return Math.abs(x) < ZERO_EPS_POS_M && Math.abs(y) < ZERO_EPS_POS_M &&
//                Math.abs(z) < ZERO_EPS_POS_M && Math.abs(yaw) < ZERO_EPS_YAW_RAD;
//    }
//
//    private static double wrap(double a) {
//        while (a > Math.PI) a -= 2 * Math.PI;
//        while (a < -Math.PI) a += 2 * Math.PI;
//        return a;
//    }
//
//    private static final class Gates {
//        double gatePos_m, gateYaw_rad;
//    }
//
//    private static Gates gatesFrom(double sx_m, double sy_m, double syaw_rad) {
//        double sigmaPos = Math.hypot(sx_m, sy_m);
//        double sigmaYaw = Math.abs(syaw_rad);
//        Gates g = new Gates();
//        g.gatePos_m = clamp(3.0 * sigmaPos, GATE_POS_MIN_M, GATE_POS_MAX_M);
//        g.gateYaw_rad = clamp(3.0 * sigmaYaw, GATE_YAW_MIN_RAD, GATE_YAW_MAX_RAD);
//        return g;
//    }
//
//    private static double clamp(double v, double lo, double hi) {
//        return Math.max(lo, Math.min(hi, v));
//    }
//
//    private static boolean isFinite(double v) { return !Double.isNaN(v) && !Double.isInfinite(v); }
//    private static boolean isFinitePose(Pose3D p) {
//        if (p == null) return false;
//        double x = p.getPosition().x;
//        double y = p.getPosition().y;
//        double yaw = p.getOrientation().getYaw(AngleUnit.RADIANS);
//        return isFinite(x) && isFinite(y) && isFinite(yaw);
//    }
//
//    private void addTelemetry(double xM, double yM, double h, LLResult r) {
//        tele.addLine("--- StateEstimator [Pedro Localizer] ---")
//                .addData("Pose_fused [in | deg]", "(%.2f, %.2f) | %.1f°",
//                        fusedIn.getX(), fusedIn.getY(), Math.toDegrees(fusedIn.getHeading()))
//                .addData("Pose_odom [m | deg]", "(%.3f, %.3f) | %.1f°",
//                        xM, yM, Math.toDegrees(h))
//                .addData("Vel [in/s | deg/s]", "vx=%.2f vy=%.2f ω=%.1f",
//                        velIn.getX(), velIn.getY(), Math.toDegrees(velIn.getHeading()));
//
//        boolean fresh = (r != null && r.isValid());
//
//        tele.addData("LL fresh/valid/accepted", "%b / %b / %b",
//                        fresh, vision.valid, vision.accepted)
//                .addData("LL age[ms] tags/span/avgDist", "%d | %d / %.2f / %.2f",
//                        vision.lastAgeMs, vision.lastTagCount, vision.lastSpan, vision.lastAvgDist)
//                .addData("LL pose [m | deg]", "(%s, %s) | %s",
//                        toStr(visionPose.llX), toStr(visionPose.llY),
//                        toStrDeg(visionPose.llTheta))
//                .addData("Residuals [m | deg]", "dx=%.3f dy=%.3f dθ=%.2f°",
//                        residual.x_m, residual.y_m, Math.toDegrees(residual.theta_rad))
//                .addData("Gates [m | deg]", "pos<%.2f yaw<%.1f°",
//                        residual.gatePos_m, Math.toDegrees(residual.gateYaw_rad))
//                .addData("Frames seen/accepted", "%d / %d",
//                        vision.framesWithVision, vision.framesAccepted);
//    }
//
//    @SuppressLint("DefaultLocale")
//    private static String toStr(double v) {
//        return (Double.isNaN(v) || Double.isInfinite(v)) ? "NaN" : String.format("%.3f", v);
//    }
//    @SuppressLint("DefaultLocale")
//    private static String toStrDeg(double v) {
//        return (Double.isNaN(v) || Double.isInfinite(v)) ? "NaN" : String.format("%.1f", Math.toDegrees(v));
//    }
//}
