package org.firstinspires.ftc.teamcode.localisation;

import static org.firstinspires.ftc.teamcode.localisation.LocalisationConstants.*;
import static org.firstinspires.ftc.teamcode.localisation.Constants.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.localisation.Constants.kPos;
import static org.firstinspires.ftc.teamcode.localisation.Constants.kTheta;

import android.annotation.SuppressLint;

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

import java.util.ArrayDeque;
import java.util.Arrays;

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
    private boolean seeded = false;
    private boolean firstLockDone = false;
    private int lastTagCount = 0;
    private double lastSpan = 0, lastAvgDist = 0;
    private boolean residStale = true;
    private final ArrayDeque<Double> omegaHist = new ArrayDeque<>(OMEGA_WINDOW);
    private long stationarySinceMs = -1;


    // Constructor
    public StateEstimator(OpMode opmode, GoBildaPinpointDriver pinpoint, AprilTagLocalizerLimelight aprilTagLocalizer) {
        this.opmode = opmode;
        this.pinpoint = pinpoint;
        this.aprilTagLocalizer = aprilTagLocalizer;
    }

    public void seedFieldPose(double x, double y, double hRad) {
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.METER, x, y, AngleUnit.RADIANS, hRad));
        pinpoint.setHeading(hRad, AngleUnit.RADIANS);
        seeded = true;
        firstLockDone = true;
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
            aprilTagLocalizer.update(Math.toDegrees(getFusedHeading(AngleUnit.RADIANS)));
            LLResult r = aprilTagLocalizer.getResult();

            if (!seeded && isUsableVision(r)) {
                Pose3D p = r.getBotpose_MT2();
                seedFieldPose(p.getPosition().x, p.getPosition().y,
                        p.getOrientation().getYaw(AngleUnit.RADIANS));
            }

            if (r!=null){
                lastTagCount = r.getBotposeTagCount();
                lastSpan = r.getBotposeSpan();
                lastAvgDist = r.getBotposeAvgDist();
            }
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

    public Pose2D getFieldPose(){
        if (fallbackMode) return getPose();

        Pose2D est = getPose();
        LLResult r = aprilTagLocalizer.getResult();
        boolean fresh = r!=null && r.isValid()
                && aprilTagLocalizer.getMillisSinceLastUpdate() <= LL_STALE_MS;
        Pose3D p = fresh ? r.getBotpose_MT2() : null;

        visionValid = fresh && p!=null && !isZeroPose(p);

        if (visionValid){
            double llX = p.getPosition().x, llY = p.getPosition().y;
            double llTh = p.getOrientation().getYaw(AngleUnit.RADIANS);
            llX_last=llX; llY_last=llY; llTheta_last=llTh; framesWithVision++;

            double dx = llX - est.getX(DistanceUnit.METER);
            double dy = llY - est.getY(DistanceUnit.METER);
            double dTh= angleWrap(llTh - est.getHeading(AngleUnit.RADIANS));
            residX=dx; residY=dy; residTheta=dTh; residStale=false;

            Mt2Quality q = readQuality(r);
            Gates g = gatesFrom(q);

            boolean gate = Math.hypot(dx,dy) < g.gatePos_m && Math.abs(dTh) < g.gateYaw_rad;
            if (gate){
                appliedX = kPos*dx; appliedY = kPos*dy; appliedTheta = kTheta*dTh;
                est = new Pose2D(DistanceUnit.METER,
                        est.getX(DistanceUnit.METER) + appliedX,
                        est.getY(DistanceUnit.METER) + appliedY,
                        AngleUnit.RADIANS,
                        angleWrap(est.getHeading(AngleUnit.RADIANS) + appliedTheta));
                visionAccepted=true; framesAccepted++;
                lastVisionAcceptMs=System.currentTimeMillis(); firstLockDone=true;
            } else {
                // Stationary snap with filtered omega
                ChassisSpeeds vf = getChassisSpeedsField();
                double omegaRaw = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
                omegaRaw = clamp(omegaRaw, -OMEGA_CLAMP_RADPS, OMEGA_CLAMP_RADPS);
                if (omegaHist.size()==OMEGA_WINDOW) omegaHist.removeFirst();
                omegaHist.addLast(omegaRaw);
                double omegaFilt = median(omegaHist);

                boolean slow = Math.hypot(vf.vxMetersPerSecond, vf.vyMetersPerSecond) < V_STAT_MPS
                        && Math.abs(omegaFilt) < OMEGA_STAT_RADPS;

                long now = System.currentTimeMillis();
                if (slow){ if (stationarySinceMs<0) stationarySinceMs=now; }
                else { stationarySinceMs=-1; }

                boolean held = stationarySinceMs>0 && (now-stationarySinceMs) >= STATIONARY_HOLD_MS;
                boolean goodSnap = Math.hypot(dx,dy) < g.snapPos_m && Math.abs(dTh) < g.snapYaw_rad;

                if (held && goodSnap){
                    pinpoint.setPosition(new Pose2D(
                            DistanceUnit.METER, llX, llY, AngleUnit.RADIANS, llTh)
                    );
                    pinpoint.setHeading(llTh, AngleUnit.RADIANS);
                    est = new Pose2D(DistanceUnit.METER, llX, llY, AngleUnit.RADIANS, llTh);
                    appliedX=dx; appliedY=dy; appliedTheta=dTh;
                    visionAccepted=true; framesAccepted++; lastVisionAcceptMs=now; seeded=true;
                    firstLockDone=true;
                } else {
                    visionAccepted=false; appliedX=appliedY=appliedTheta=0;
                }
            }
        } else {
            visionAccepted=false; residStale=true; appliedX=appliedY=appliedTheta=0;
        }
        return est;
    }


    public Pose2D getFusedPose() { return lastFieldPose; }
    public double getFusedHeading(AngleUnit angleUnit) {return lastFieldPose.getHeading(angleUnit);}

    public void reset() {
        pinpoint.resetPosAndIMU();
    }

    @SuppressLint("DefaultLocale")
    public void addTelemetry() {
        // Odometry-frame pose (Pinpoint)
        Pose2D odo = getPose();
        double ox = odo.getX(DistanceUnit.METER);
        double oy = odo.getY(DistanceUnit.METER);
        double ohRad = odo.getHeading(AngleUnit.RADIANS);

        // Velocities
        ChassisSpeeds vf = getChassisSpeedsField();
        ChassisSpeeds vr = getChassisSpeedsRobot();

        // Headings in degrees
        double ohDeg = Math.toDegrees(ohRad);

        // Angular rates in deg/s
        double omegaFieldDeg = Math.toDegrees(vf.omegaRadiansPerSecond);
        double omegaRobotDeg = Math.toDegrees(vr.omegaRadiansPerSecond);

        opmode.telemetry.addLine(
                fallbackMode ? "--- StateEstimator Fallback ---" : "--- StateEstimator ---"
        );

        opmode.telemetry.addData("Pinpoint pose (odom)",
                "(%.3f, %.3f) m | %.1f°", ox, oy, ohDeg);

        opmode.telemetry.addData("v_field [m/s, deg/s]",
                "vx=%.3f vy=%.3f ω=%.1f",
                vf.vxMetersPerSecond, vf.vyMetersPerSecond, omegaFieldDeg);

        opmode.telemetry.addData("v_robot [m/s, deg/s]",
                "vx=%.3f vy=%.3f ω=%.1f",
                vr.vxMetersPerSecond, vr.vyMetersPerSecond, omegaRobotDeg);

        if (!fallbackMode) {
            Pose2D est = lastFieldPose;
            double ex = est.getX(DistanceUnit.METER);
            double ey = est.getY(DistanceUnit.METER);
            double ehRad = est.getHeading(AngleUnit.RADIANS);
            double ehDeg = Math.toDegrees(ehRad);

            opmode.telemetry.addData("Pose (field)",
                    "(%.3f, %.3f) m | %.1f°", ex, ey, ehDeg);

            opmode.telemetry.addData("Seeded/FirstLock",
                    "%b / %b",
                    seeded,
                    firstLockDone);
            opmode.telemetry.addData("Last vision age [ms]",
                    (aprilTagLocalizer.getMillisSinceLastUpdate()));

            opmode.telemetry.addData("LL valid / accepted", "%b / %b",
                    visionValid,
                    visionAccepted);
            opmode.telemetry.addData("LL pose (field)",
                    "(%.3f, %.3f) m | %.1f°", llX_last, llY_last,
                    Math.toDegrees(llTheta_last));

            double posErr = Math.hypot(residX, residY);
            opmode.telemetry.addData("LL residual",
                    residStale ? "stale" :
                            String.format("dx=%.3f dy=%.3f dθ=%.1f° | ‖pos‖=%.3f m",
                                    residX, residY, Math.toDegrees(residTheta), posErr));

            LLResult rr = aprilTagLocalizer.getResult();
            if (rr != null && rr.isValid()) {
                Gates g = gatesFrom(readQuality(rr));
                opmode.telemetry.addData("Gates",
                        "gate<%.2f m & <%.1f° | snap<%.2f m & <%.1f°",
                        g.gatePos_m, Math.toDegrees(g.gateYaw_rad),
                        g.snapPos_m, Math.toDegrees(g.snapYaw_rad));
            }

            opmode.telemetry.addData("Applied",
                    "x=%.3f y=%.3f θ=%.1f°",
                    appliedX, appliedY, Math.toDegrees(appliedTheta));

            long sinceMs = lastVisionAcceptMs < 0 ? -1 :
                    (System.currentTimeMillis() - lastVisionAcceptMs);
            opmode.telemetry.addData("Since last accept [ms] / counts",
                    "%d | seen=%d accepted=%d", sinceMs, framesWithVision, framesAccepted);

            opmode.telemetry.addData("LL quality",
                    "tags=%d span=%.2f m avgDist=%.2f m",
                    lastTagCount, lastSpan, lastAvgDist);
            opmode.telemetry.addData("Heading [deg]", "%.1f",
                    Math.toDegrees(getHeading()));
        }
    }

    // Utils or helpers
    private static double angleWrap(double a) {
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }

    // utils
    private static double clamp(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); }
    private static double median(ArrayDeque<Double> d){
        double[] a = d.stream().mapToDouble(x->x).toArray();
        Arrays.sort(a);
        int n=a.length; return n==0?0:(n%2==1?a[n/2]:0.5*(a[n/2-1]+a[n/2]));
    }
    private static boolean isZeroPose(Pose3D p){
        if (p==null) return true;
        double x=p.getPosition().x, y=p.getPosition().y, z=p.getPosition().z;
        double yaw=p.getOrientation().getYaw(AngleUnit.RADIANS);
        return Math.abs(x)<ZERO_EPS_POS_M
                && Math.abs(y)<ZERO_EPS_POS_M
                && Math.abs(z)<ZERO_EPS_POS_M
                && Math.abs(yaw)<ZERO_EPS_YAW_RAD;
    }
    private static final class Mt2Quality {
        int tagCount; double span_m, avgDist_m, avgArea, sx_m, sy_m, syaw_rad;
    }
    private static Mt2Quality readQuality(LLResult r){
        Mt2Quality q = new Mt2Quality();
        q.tagCount = r.getBotposeTagCount();
        q.span_m = r.getBotposeSpan();
        q.avgDist_m = r.getBotposeAvgDist();
        q.avgArea =r.getBotposeAvgArea();

        double[] std = r.getStddevMt2();
        if (std != null && std.length >= 6) {
            q.sx_m = std[0];
            q.sy_m = std[1];
            q.syaw_rad = std[5];
        }
        if (!(q.sx_m > 0)) q.sx_m = 0.25;
        if (!(q.sy_m > 0)) q.sy_m = 0.25;
        if (!(q.syaw_rad > 0)) q.syaw_rad = Math.toRadians(10);
        return q;
    }

    private boolean isUsableVision(LLResult r){
        if (r==null || !r.isValid()) return false;
        if (aprilTagLocalizer.getMillisSinceLastUpdate() > LL_STALE_MS) return false;
        Pose3D p = r.getBotpose_MT2(); if (isZeroPose(p)) return false;

        Mt2Quality q = readQuality(r);
        if (q.tagCount < MIN_TAGS) return false;

        double sigmaPos = Math.hypot(q.sx_m, q.sy_m);
        boolean multi = q.tagCount >= 2;
        boolean singleOk = (!multi) && (3.0*sigmaPos <= SINGLE_TAG_3SIGMA_POS_MAX_M);
        if (!(multi || singleOk)) return false;

        return !(q.avgArea < MIN_AVG_AREA);
    }
    private static final class Gates { double gatePos_m, gateYaw_rad, snapPos_m, snapYaw_rad; }
    private static Gates gatesFrom(Mt2Quality q){
        double sigmaPos = Math.hypot(q.sx_m, q.sy_m);
        double sigmaYaw = Math.abs(q.syaw_rad);
        Gates g=new Gates();
        g.gatePos_m = clamp(3.0*sigmaPos, GATE_POS_MIN_M, GATE_POS_MAX_M);
        g.gateYaw_rad = clamp(3.0*sigmaYaw, GATE_YAW_MIN_RAD, GATE_YAW_MAX_RAD);
        g.snapPos_m = clamp(4.0*sigmaPos, g.gatePos_m, SNAP_POS_MAX_M);
        g.snapYaw_rad = clamp(4.0*sigmaYaw, g.gateYaw_rad, SNAP_YAW_MAX_RAD);
        return g;
    }
}