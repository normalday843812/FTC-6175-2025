package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class WebcamPinpointLocaliser implements Localizer {
    private static final boolean SWAP_FOR_FTC_FIELD = true;

    // TODO: Tune
    private static final double CLOSE_RANGE_IN = 18.0;
    private static final double FAR_RANGE_IN = 90.0;
    private static final double WEIGHT_NEAR = 0.65;
    private static final double WEIGHT_FAR = 0.15;

    private static final double MAX_TRANSLATION_JUMP_IN = 6.0;
    private static final double MAX_HEADING_JUMP_RAD = Math.toRadians(20.0);

    private static final long MAX_TAG_AGE_MS = 200;

    private final PinpointLocalizer odoLocalizer;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor tagProcessor;

    private Pose fusedPose = new Pose(0,0,0);
    private Pose fusedVelocity = new Pose(0,0,0);
    private double totalHeading = 0;

    private long lastTagTimestampMs = 0;
    private long lastUpdateTimeNs = 0;
    public WebcamPinpointLocaliser(
            HardwareMap hw,
            PinpointConstants pinpointConstants,
            Pose startPose,
            String webcamName,
            Position cameraPositionOnRobot,
            YawPitchRollAngles cameraOrientationOnRobot
    ) {
        odoLocalizer = new PinpointLocalizer(hw, pinpointConstants, startPose);
        fusedPose = startPose;

        AprilTagProcessor.Builder atBuilder = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setCameraPose(cameraPositionOnRobot, cameraOrientationOnRobot);
        tagProcessor = atBuilder.build();

        // 3) VisionPortal with the webcam
        VisionPortal.Builder vpBuilder = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, webcamName))
                .addProcessor(tagProcessor);
        visionPortal = vpBuilder.build();

        lastUpdateTimeNs = System.nanoTime();
    }

    // Constructor with defaults (camera at robot center, forward facing, horizontal)
    public WebcamPinpointLocaliser(
            HardwareMap hw,
            PinpointConstants pinpointConstants,
            Pose startPose,
            String webcamName
    ) {
        this(
                hw,
                pinpointConstants,
                startPose,
                webcamName,
                new Position(DistanceUnit.INCH, 0, 0, 0, 0),
                new YawPitchRollAngles(AngleUnit.RADIANS, 0, -Math.PI / 2.0, 0, 0)
        );
    }

    @Override
    public Pose getPose() {
        return fusedPose;
    }

    @Override
    public Pose getVelocity() {
        return fusedVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        return fusedVelocity.getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        odoLocalizer.setStartPose(setStart);
        fusedPose = setStart;
        totalHeading = 0;
    }

    @Override
    public void setPose(Pose setPose) {
        odoLocalizer.setPose(setPose);
        fusedPose = setPose;
    }

    @Override
    public void update() {
        odoLocalizer.update();
        Pose odomPose = odoLocalizer.getPose();
        fusedVelocity = odoLocalizer.getVelocity();

        long nowNs = System.nanoTime();
        long nowMs = nowNs / 1_000_000L;
        long dtNs = lastUpdateTimeNs == 0 ? 0 : (nowNs - lastUpdateTimeNs);
        lastUpdateTimeNs = nowNs;

        Pose tagPose = computeBestAprilTagPose();
        boolean tagFresh = (tagPose != null) && (nowMs - lastTagTimestampMs <= MAX_TAG_AGE_MS);
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return odoLocalizer.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return odoLocalizer.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return odoLocalizer.getTurningMultiplier();
    }

    @Override
    public void resetIMU() {
        odoLocalizer.resetIMU();
    }

    @Override
    public double getIMUHeading() {
        return odoLocalizer.getIMUHeading();
    }

    @Override
    public boolean isNAN() {
        return false; // TODO: implement correctly
    }

    private Pose computeBestAprilTagPose() {
        List<AprilTagDetection> list = tagProcessor.getDetections();
        if (list == null || list.isEmpty()) return null;

        List<AprilTagDetection> usable = new ArrayList<>();
        for (AprilTagDetection d : list) {
            if (d.metadata == null) continue;
            if (d.robotPose != null) {
                usable.add(d);
            }
        }
        if (usable.isEmpty()) return null;

        lastTagTimestampMs = System.currentTimeMillis();

        double sumW = 0, sx = 0, sy = 0, sh = 0;

        for (AprilTagDetection d : usable) {
            double xr = d.robotPose.getPosition().x; // FTC field: +X Right
            double yf = d.robotPose.getPosition().y; // FTC field: +Y Forward
            double yaw = d.robotPose.getOrientation().getYaw(AngleUnit.RADIANS); // CCW+

            Pose pPedro = ftcFieldToPedroPose(xr, yf, yaw);

            double range = Math.hypot(xr, yf);
            double w = 1.0 / Math.max(range, 1.0); // avoid div by zero
            sumW += w;
            sx += w * pPedro.getX();
            sy += w * pPedro.getY();
            sh += w * wrapAngle(pPedro.getHeading());
        }

        if (sumW <= 0) return null;

        return new Pose(sx / sumW, sy / sumW, wrapAngle(sh / sumW));
    }

    private boolean isReasonableJump(Pose odom, Pose tag) {
        double dx = tag.getX() - odom.getX();
        double dy = tag.getY() - odom.getY();
        double d = Math.hypot(dx, dy);
        double dh = Math.abs(angleDiff(tag.getHeading(), odom.getHeading()));
        return d <= MAX_TRANSLATION_JUMP_IN && dh <= MAX_HEADING_JUMP_RAD;
    }

    private Pose blendPoses(Pose aOdom, Pose bTag, double wOdom, double wTag) {
        double x = wOdom * aOdom.getX() + wTag * bTag.getX();
        double y = wOdom * aOdom.getY() + wTag * bTag.getY();
        double h = wrapAngle(wOdom * wrapAngle(aOdom.getHeading()) + wTag * wrapAngle(bTag.getHeading()));
        return new Pose(x, y, h);
    }

    private double weightFromRange(double rangeIn) {
        // Map [CLOSE_RANGE_IN .. FAR_RANGE_IN] -> [WEIGHT_NEAR .. WEIGHT_FAR], clamped
        double t = (rangeIn - CLOSE_RANGE_IN) / Math.max(FAR_RANGE_IN - CLOSE_RANGE_IN, 1e-6);
        t = clamp01(t);
        return WEIGHT_NEAR * (1 - t) + WEIGHT_FAR * t;
    }

    private static double clamp01(double v) { return Math.max(0.0, Math.min(1.0, v)); }

    private static double wrapAngle(double a) {
        while (a <= -Math.PI) a += 2 * Math.PI;
        while (a > Math.PI)   a -= 2 * Math.PI;
        return a;
    }

    private static double angleDiff(double a, double b) {
        return wrapAngle(a - b);
    }

    // Simple proxy so totalHeading accumulates smoothly even if fused heading "wraps"
    private double totalHeadingHeadingProxy() {
        return fusedPose.getHeading();
    }

    /**
     * Map FTC field pose to Pedro pose.
     * FTC field:   (xRight, yForward, yawCCW)
     * Pedro (typ): (xForward, yLeft,   headingCCW)
     * If SWAP_FOR_FTC_FIELD is true, x <- yForward, y <- +xRight (left = +y), heading same.
     * If false, we assume the same axes and pass through x <- xRight, y <- yForward.
     */
    private Pose ftcFieldToPedroPose(double xRight, double yForward, double yawRad) {
        if (SWAP_FOR_FTC_FIELD) {
            double xForward = yForward;
            double yLeft = +xRight;
            return new Pose(xForward, yLeft, yawRad);
        } else {
            return new Pose(xRight, yForward, yawRad);
        }
    }
    public void close() {
        try {
            if (visionPortal != null) visionPortal.close();
        } catch (Throwable ignored) {}
    }
}