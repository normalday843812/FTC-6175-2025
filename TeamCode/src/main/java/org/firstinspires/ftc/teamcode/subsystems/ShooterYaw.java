package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_BLUE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_RED;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.isRed;
import static org.firstinspires.ftc.teamcode.config.LLAprilTagConfig.TTL_MS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.AUTO_LOCK_DEADBAND_DEG;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.AUTO_LOCK_MAX_POWER;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.CENTER_POS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.CONTROL_LEVEL;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.KF_Q_IMU;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.KF_Q_LL;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.KF_R_IMU;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.KF_R_LL_BASE;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.LL_WEIGHT_DIST2;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MAX_POSITION;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.METERS_PER_POSE_UNIT;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MIN_POSITION;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.PIDF_COEFFICIENTS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SCAN_BAND_DEG;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SCAN_PERIOD_MS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SEEK_WINDOW_MS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SOFT_LIMIT_MARGIN;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.STABLE_HOLD_MS_DEFAULT;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.YAW_POWER;

import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.vision.LLAprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

public class ShooterYaw {
    private enum ControlState {MANUAL_HOLD, SEEK_GOAL, LOCK_GOAL, SEEK_PATTERN, LOCK_PATTERN}

    private final DcMotorEx motor;
    private final GamepadMap map;
    private final TelemetryHelper tele;
    private final LLAprilTag ll;
    private final Follower follower;
    private SubsystemMode mode = SubsystemMode.MANUAL;
    private ControlState control = ControlState.MANUAL_HOLD;
    private int targetPosition = CENTER_POS;
    private int position = 0;
    private final FilteredPIDFController pid =
            new FilteredPIDFController(PIDF_COEFFICIENTS);
    private final KalmanFilter kfIMU =
            new KalmanFilter(new KalmanFilterParameters(KF_Q_IMU, KF_R_IMU));
    private final KalmanFilter kfLL =
            new KalmanFilter(new KalmanFilterParameters(KF_Q_LL, KF_R_LL_BASE));
    private int targetId = -1;
    private double lastFusedErrDeg = 0.0;
    private int[] patternIds = null;
    private int patternIdx = 0;
    private int chosenPatternId = -1;
    private long stableHoldMsGoal = STABLE_HOLD_MS_DEFAULT;
    private final Timer stableTimer = new Timer();
    private long sweepStartMs = System.currentTimeMillis();

    public ShooterYaw(DcMotorEx shooterYawMotor,
                      LLAprilTag aprilTag,
                      GamepadMap map,
                      Follower follower,
                      OpMode opmode) {
        this.motor = shooterYawMotor;
        this.ll = aprilTag;
        this.map = map;
        this.follower = follower;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(targetPosition);
        motor.setPower(YAW_POWER);
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
        control = ControlState.MANUAL_HOLD;
        pid.reset();
        kfIMU.reset();
        kfLL.reset();
    }

    public void startAuto() {
        mode = SubsystemMode.AUTO;
        control = ControlState.SEEK_GOAL;
        pid.reset();
        kfIMU.reset();
        kfLL.reset();
        targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        sweepStartMs = System.currentTimeMillis();
        stableTimer.resetTimer();
    }

    public void seekPattern(int[] ids, long stableHoldMs) {
        patternIds = (ids == null || ids.length == 0) ? null : ids.clone();
        patternIdx = 0;
        chosenPatternId = -1;
        stableHoldMsGoal = stableHoldMs;
        if (mode != SubsystemMode.AUTO) mode = SubsystemMode.AUTO;
        control = ControlState.SEEK_PATTERN;
        pid.reset();
        kfIMU.reset();
        kfLL.reset();
        sweepStartMs = System.currentTimeMillis();
        stableTimer.resetTimer();
    }

    public boolean isPatternChosen() {
        return chosenPatternId >= 0;
    }

    public int getChosenPatternId() {
        return chosenPatternId;
    }

    public void operate() {
        if (map != null && mode == SubsystemMode.MANUAL && map.shooterYawAutoLockToggle) {
            enableTeleopAutoGoal();
        } else if (map != null && mode == SubsystemMode.AUTO && map.shooterYawAutoLockToggle) {
            disableAutoToManual();
        }
        if (map != null && map.resetShooterYaw) {
            resetShooterYaw();
            return;
        }
        position = motor.getCurrentPosition();
        if (mode == SubsystemMode.MANUAL) {
            operateManual();
        } else {
            switch (control) {
                case SEEK_GOAL:
                    targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
                    seekOne(targetId);
                    break;
                case LOCK_GOAL:
                    lockTo(targetId);
                    break;
                case SEEK_PATTERN:
                    seekPatternOnce();
                    break;
                case LOCK_PATTERN:
                    lockTo(targetId);
                    break;
                default:
                    holdHere();
                    break;
            }
        }
        addTelemetry();
    }

    private void operateManual() {
        if (map != null) {
            if ((map.shooterYaw < 0 && targetPosition > MIN_POSITION) ||
                    (map.shooterYaw > 0 && targetPosition < MAX_POSITION)) {
                targetPosition += (int) (map.shooterYaw * CONTROL_LEVEL);
            }
        }
        targetPosition = clampPosition(targetPosition);
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motor.setTargetPosition(targetPosition);
        motor.setPower(YAW_POWER);
    }

    private void enableTeleopAutoGoal() {
        mode = SubsystemMode.AUTO;
        control = ControlState.SEEK_GOAL;
        targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        pid.reset();
        kfIMU.reset();
        kfLL.reset();
        sweepStartMs = System.currentTimeMillis();
        stableTimer.resetTimer();
    }

    private void disableAutoToManual() {
        mode = SubsystemMode.MANUAL;
        control = ControlState.MANUAL_HOLD;
        holdHere();
        pid.reset();
        kfIMU.reset();
        kfLL.reset();
    }

    private void seekOne(int id) {
        Pose pose = follower.getPose();
        double headingDeg = Math.toDegrees(pose.getHeading());
        double bearingDeg = getBearingDeg(id, pose);
        double sweepOffset = SCAN_BAND_DEG * Math.sin(twoPi(System.currentTimeMillis() - sweepStartMs) / SCAN_PERIOD_MS);
        double errGeom = normalizeDeg(bearingDeg + sweepOffset - headingDeg);
        kfIMU.update(0.0, errGeom);
        LLAprilTag.YawInfo info = ll.getYawInfoForTag(id);
        if (info.fresh && info.ageMs <= TTL_MS && Double.isFinite(info.avgDeg)) {
            double errLL = -info.avgDeg;
            kfLL.update(0.0, errLL);
            control = ControlState.LOCK_GOAL;
            double fused = fuse(info.distanceM, kfIMU.getState(), kfLL.getState());
            driveYawByError(fused);
            return;
        }
        driveYawByError(kfIMU.getState());
    }

    private void seekPatternOnce() {
        if (patternIds == null || patternIds.length == 0) {
            control = ControlState.SEEK_GOAL;
            return;
        }
        int id = patternIds[patternIdx];
        Pose pose = follower.getPose();
        double headingDeg = Math.toDegrees(pose.getHeading());
        double bearingDeg = getBearingDeg(id, pose);
        double sweepOffset = SCAN_BAND_DEG * Math.sin(twoPi(System.currentTimeMillis() - sweepStartMs) / SCAN_PERIOD_MS);
        double errGeom = normalizeDeg(bearingDeg + sweepOffset - headingDeg);
        kfIMU.update(0.0, errGeom);
        LLAprilTag.YawInfo info = ll.getYawInfoForTag(id);
        if (info.fresh && info.ageMs <= TTL_MS && Double.isFinite(info.avgDeg)) {
            double errLL = -info.avgDeg;
            kfLL.update(0.0, errLL);
            if (stableTimer.getElapsedTime() >= stableHoldMsGoal) {
                chosenPatternId = id;
                targetId = id;
                control = ControlState.LOCK_PATTERN;
            }
        } else {
            stableTimer.resetTimer();
        }
        if (System.currentTimeMillis() - sweepStartMs > SEEK_WINDOW_MS) {
            patternIdx = (patternIdx + 1) % patternIds.length;
            sweepStartMs = System.currentTimeMillis();
        }
        driveYawByError(kfIMU.getState());
    }

    private void lockTo(int id) {
        Pose pose = follower.getPose();
        double headingDeg = Math.toDegrees(pose.getHeading());
        double bearingDeg = getBearingDeg(id, pose);
        double errGeom = normalizeDeg(bearingDeg - headingDeg);
        kfIMU.update(0.0, errGeom);
        LLAprilTag.YawInfo info = ll.getYawInfoForTag(id);
        double fused = kfIMU.getState();
        if (info.fresh && info.ageMs <= TTL_MS && Double.isFinite(info.avgDeg)) {
            double errLL = -info.avgDeg;
            kfLL.update(0.0, errLL);
            fused = fuse(info.distanceM, kfIMU.getState(), kfLL.getState());
        }
        lastFusedErrDeg = fused;
        driveYawByError(fused);
    }

    private void driveYawByError(double errDeg) {
        if (Math.abs(errDeg) < AUTO_LOCK_DEADBAND_DEG) {
            stopPower();
            return;
        }
        pid.updateError(errDeg);
        double power = clamp(pid.run(), -AUTO_LOCK_MAX_POWER, AUTO_LOCK_MAX_POWER);
        position = motor.getCurrentPosition();
        if ((position >= MAX_POSITION - SOFT_LIMIT_MARGIN && power > 0) ||
                (position <= MIN_POSITION + SOFT_LIMIT_MARGIN && power < 0)) {
            stopPower();
            return;
        }
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(power);
    }

    private void stopPower() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0.0);
    }

    public boolean isLockedOnTarget() {
        return mode == SubsystemMode.AUTO && Math.abs(lastFusedErrDeg) < AUTO_LOCK_DEADBAND_DEG;
    }

    public void center() {
        targetPosition = clampPosition(CENTER_POS);
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motor.setTargetPosition(targetPosition);
        motor.setPower(YAW_POWER);
    }

    public void lockAllianceGoal() {
        mode = SubsystemMode.AUTO;
        control = ControlState.LOCK_GOAL;
        targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        patternIds = null;
        chosenPatternId = -1;
        pid.reset();
        kfIMU.reset();
        kfLL.reset();
    }

    public void resetShooterYaw() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = 0;
        position = 0;
        pid.reset();
        kfIMU.reset();
        kfLL.reset();
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(targetPosition);
        motor.setPower(YAW_POWER);
    }

    private void holdHere() {
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        position = motor.getCurrentPosition();
        targetPosition = position;
        motor.setTargetPosition(targetPosition);
        motor.setPower(YAW_POWER);
    }

    private static double getBearingDeg(int tagId, Pose robotPose) {
        AprilTagMetadata md = AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(tagId);
        if (md == null) return 0.0;
        VectorF p = md.fieldPosition;
        double tagXm = p.get(0);
        double tagYm = p.get(1);
        double rxm = robotPose.getX() * METERS_PER_POSE_UNIT;
        double rym = robotPose.getY() * METERS_PER_POSE_UNIT;
        double dx = tagXm - rxm;
        double dy = tagYm - rym;
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    private int clampPosition(int pos) {
        return (int) clamp(pos, MIN_POSITION, MAX_POSITION);
    }

    private static double normalizeDeg(double deg) {
        double x = deg % 360.0;
        if (x > 180.0) x -= 360.0;
        if (x <= -180.0) x += 360.0;
        return x;
    }

    private static double fuse(double distanceM, double imuDeg, double llDeg) {
        double w = 1.0 / (1.0 + LL_WEIGHT_DIST2 * distanceM * distanceM);
        if (w < 0.0) w = 0.0;
        if (w > 1.0) w = 1.0;
        return (1.0 - w) * imuDeg + w * llDeg;
    }

    private static double twoPi(long ms) {
        return 2.0 * Math.PI * (ms / 1000.0);
    }

    // For telemetry panel
    private void addTelemetry() {
        if (!TELEMETRY_ENABLED) return;
        tele.addLine("=== SHOOTER YAW ===")
                .addData("Mode", mode::name)
                .addData("Ctrl", control::name)
                .addData("AllianceRed", "%b", isRed)
                .addData("TargetId", "%d", targetId)
                .addData("CurPos", "%d", position)
                .addData("TgtPos", "%d", targetPosition)
                .addData("KF_IMU", "%.2f", kfIMU.getState())
                .addData("KF_LL", "%.2f", kfLL.getState())
                .addData("FusedErr", "%.2f", lastFusedErrDeg);
    }
}