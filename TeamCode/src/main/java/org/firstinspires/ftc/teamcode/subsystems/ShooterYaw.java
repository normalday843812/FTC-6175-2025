package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_BLUE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_RED;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.isRed;
import static org.firstinspires.ftc.teamcode.config.LLAprilTagConfig.TTL_MS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.*;

import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.control.LowPassFilter;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
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

    private enum ControlState {
        MANUAL_HOLD,
        AUTO_WORLD_LOCK,
        SEEK_PATTERN
    }

    private final DcMotorEx motor;
    private final GamepadMap map;
    private final TelemetryHelper tele;
    private final LLAprilTag ll;
    private final Follower follower;

    private SubsystemMode mode = SubsystemMode.MANUAL;
    private ControlState control = ControlState.MANUAL_HOLD;

    private int reportedTicks = 0;
    private final double ticksPerDeg = TICKS_PER_DEG_INIT;
    private final double degPerTick = 1.0 / TICKS_PER_DEG_INIT;

    private double worldTargetRad = 0.0;
    private boolean worldTargetInitialized = false;

    private int targetId = -1;

    private final FilteredPIDFController posPid = new FilteredPIDFController(PIDF_COEFFICIENTS);

    private final LowPassFilter velLP = new LowPassFilter(VEL_LP_ALPHA);
    private long lastVelNs;
    private int lastVelTicks;

    private double iAccum = 0.0;

    private long lastLLFreshMs = 0L;

    private int[] patternIds = null;
    private int patternIdx = 0;
    private int chosenPatternId = -1;
    private long stableHoldMsGoal = STABLE_HOLD_MS_DEFAULT;
    private final Timer stableTimer = new Timer();
    private long sweepStartMs = System.currentTimeMillis();

    private double lastErrorDeg = 0.0;
    private double lastRobotOmegaRad = 0.0;
    private double lastCmdPower = 0.0;
    private int lastDesiredTicks = 0;

    private long lastLoopNs = System.nanoTime();
    private int targetTicksSetpoint;

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

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        posPid.reset();
        posPid.setD(0.0);

        worldTargetInitialized = false;
        targetTicksSetpoint = CENTER_POS;
        lastVelTicks = 0;
        lastVelNs = System.nanoTime();

    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
        control = ControlState.MANUAL_HOLD;
        posPid.reset();
        posPid.setD(0.0);
        worldTargetInitialized = false;
        targetTicksSetpoint = getCurrentTicks();
        iAccum = 0.0;
    }

    public void startAuto() {
        mode = SubsystemMode.AUTO;
        control = ControlState.AUTO_WORLD_LOCK;
        targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        posPid.reset();
        posPid.setD(0.0);
        worldTargetInitialized = false;
        targetTicksSetpoint = getCurrentTicks();
        lastLLFreshMs = 0L;
        iAccum = 0.0;
    }

    public void seekPattern(int[] ids, long stableHoldMs) {
        patternIds = (ids == null || ids.length == 0) ? null : ids.clone();
        patternIdx = 0;
        chosenPatternId = -1;
        stableHoldMsGoal = stableHoldMs <= 0 ? STABLE_HOLD_MS_DEFAULT : stableHoldMs;

        if (mode != SubsystemMode.AUTO) startAuto();
        control = ControlState.SEEK_PATTERN;
        sweepStartMs = System.currentTimeMillis();
        stableTimer.resetTimer();

    }

    public boolean isPatternChosen() {
        return chosenPatternId >= 0;
    }

    public int getChosenPatternId() {
        return chosenPatternId;
    }

    public boolean isLockedOnTarget() {
        return mode == SubsystemMode.AUTO && control ==
                ControlState.AUTO_WORLD_LOCK && worldTargetInitialized;
    }

    public void center() {
        targetTicksSetpoint = CENTER_POS;
    }

    public void lockAllianceGoal() {
        mode = SubsystemMode.AUTO;
        control = ControlState.AUTO_WORLD_LOCK;
        targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        posPid.reset();
        posPid.setD(0.0);
        worldTargetInitialized = false;
        lastLLFreshMs = 0L;
        iAccum = 0.0;
    }

    public void resetShooterYaw() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        reportedTicks = 0;
        targetTicksSetpoint = CENTER_POS;
        posPid.reset();
        posPid.setD(0.0);
        worldTargetInitialized = false;
        iAccum = 0.0;
        lastVelTicks = 0;
        lastVelNs = System.nanoTime();
    }

    public void operate() {
        if (map != null && mode == SubsystemMode.MANUAL && map.shooterYawAutoLockToggle) {
            enableAutoLock();
        } else if (map != null && mode == SubsystemMode.AUTO && map.shooterYawAutoLockToggle) {
            disableAutoToManual();
        }
        if (map != null && map.resetShooterYaw) {
            resetShooterYaw();
            return;
        }

        reportedTicks = getCurrentTicks();
        long nowMs = System.currentTimeMillis();

        if (mode == SubsystemMode.MANUAL) {
            operateManual();
        } else {
            switch (control) {
                case SEEK_PATTERN:
                    seekPatternStep(nowMs);
                    break;
                case AUTO_WORLD_LOCK:
                default:
                    operateAutoWorldLock(nowMs);
                    break;
            }
        }

        addTelemetry();

    }

    private void operateManual() {
        if (map != null) {
            if ((map.shooterYaw < 0 && targetTicksSetpoint > MIN_POSITION) ||
                    (map.shooterYaw > 0 && targetTicksSetpoint < MAX_POSITION)) {
                targetTicksSetpoint += (int) (map.shooterYaw * CONTROL_LEVEL);
            }
        }
        targetTicksSetpoint = clampPosition(targetTicksSetpoint);
        drivePositionToTicks(targetTicksSetpoint, 0.0);
        control = ControlState.MANUAL_HOLD;
    }

    private void operateAutoWorldLock(long nowMs) {
        double robotHeadingRad = getRobotYawRad();
        double robotOmegaRad = getRobotOmegaRad();
        lastRobotOmegaRad = robotOmegaRad;

        LLAprilTag.YawInfo info = ll.getYawInfoForTag(resolveTargetId());
        boolean llValid = isLlValid(info);

        if (llValid && Math.abs(robotOmegaRad) <= OMEGA_LL_UPDATE_MAX_RAD) {
            double yawErrRad = Math.toRadians(-info.rawDeg);
            double gimbalWorldRad = robotHeadingRad + getGimbalRelativeRad();
            double newWorldTarget = normalizeSignedRad(gimbalWorldRad - yawErrRad);

            if (!worldTargetInitialized) {
                worldTargetRad = newWorldTarget;
                worldTargetInitialized = true;
            } else {
                worldTargetRad = blendAngles(worldTargetRad, newWorldTarget, TARGET_BLEND_ALPHA);
            }
            lastLLFreshMs = nowMs;
        }

        if (!worldTargetInitialized) {
            worldTargetRad = robotHeadingRad + getGimbalRelativeRad();
            worldTargetInitialized = true;
        }

        double desiredRelRad = normalizeSignedRad(worldTargetRad - robotHeadingRad);

        int desiredTicks = wrapToNearestTicks(desiredRelRad, reportedTicks);
        if (REACQUIRE_SCAN_ENABLED && (nowMs - lastLLFreshMs) >= REACQUIRE_AFTER_MS) {
            desiredTicks = addSafeScan(desiredTicks, nowMs);
        }
        desiredTicks = enforceSoftLimitVector(desiredTicks, reportedTicks);
        lastDesiredTicks = desiredTicks;

        double ffDegPerSec = -Math.toDegrees(robotOmegaRad);
        drivePositionToTicks(desiredTicks, ffDegPerSec);

        lastErrorDeg = (desiredTicks - reportedTicks) * degPerTick;

    }

    private void seekPatternStep(long nowMs) {
        if (patternIds == null || patternIds.length == 0) {
            control = ControlState.AUTO_WORLD_LOCK;
            return;
        }

        int id = patternIds[patternIdx];

        Pose pose = follower != null ? follower.getPose() : new Pose();
        double headingDeg = Math.toDegrees(pose.getHeading());
        double bearingDeg = getBearingDeg(id, pose);
        double baseErrDeg = normalizeDeg(bearingDeg - headingDeg);

        int centerTicks = clampPosition(reportedTicks - (int) Math.round(baseErrDeg * ticksPerDeg));
        int desired = addSafeScan(centerTicks, nowMs);

        desired = enforceSoftLimitVector(desired, reportedTicks);
        lastDesiredTicks = desired;

        double robotOmegaRad = getRobotOmegaRad();
        double ffDegPerSec = -Math.toDegrees(robotOmegaRad);
        drivePositionToTicks(desired, ffDegPerSec);

        LLAprilTag.YawInfo info = ll.getYawInfoForTag(id);
        if (isLlValid(info)) {
            if (stableTimer.getElapsedTime() >= stableHoldMsGoal) {
                chosenPatternId = id;
                targetId = id;
                double yawErrRad = Math.toRadians(-info.rawDeg);
                double gimbalWorldRad = getRobotYawRad() + getGimbalRelativeRad();
                worldTargetRad = normalizeSignedRad(gimbalWorldRad - yawErrRad);
                worldTargetInitialized = true;
                lastLLFreshMs = nowMs;
                control = ControlState.AUTO_WORLD_LOCK;
                return;
            }
        } else {
            stableTimer.resetTimer();
        }

        if (nowMs - sweepStartMs > SEEK_WINDOW_MS) {
            patternIdx = (patternIdx + 1) % patternIds.length;
            sweepStartMs = nowMs;
        }

    }

    private void enableAutoLock() {
        mode = SubsystemMode.AUTO;
        control = ControlState.AUTO_WORLD_LOCK;
        targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        posPid.reset();
        posPid.setD(0.0);
        worldTargetInitialized = false;
        lastLLFreshMs = 0L;
        iAccum = 0.0;
    }

    private void disableAutoToManual() {
        mode = SubsystemMode.MANUAL;
        control = ControlState.MANUAL_HOLD;
        posPid.reset();
        posPid.setD(0.0);
        worldTargetInitialized = false;
        iAccum = 0.0;
    }

    private boolean isLlValid(LLAprilTag.YawInfo info) {
        if (info == null) return false;
        if (!info.fresh) return false;
        if (info.ageMs > TTL_MS) return false;
        return Double.isFinite(info.rawDeg);
    }

    private int addSafeScan(int centerTicks, long nowMs) {
        int ampTicks = (int) Math.round(ticksPerDeg * SCAN_BAND_DEG);
        int leftRoom = (MAX_POSITION - SOFT_LIMIT_MARGIN) - centerTicks;
        int rightRoom = centerTicks - (MIN_POSITION + SOFT_LIMIT_MARGIN);
        int maxAmp = Math.max(0, Math.min(Math.min(leftRoom, rightRoom), ampTicks));
        if (maxAmp < MIN_SCAN_AMPLITUDE_TICKS) return centerTicks;

        long elapsed = nowMs - sweepStartMs;
        double phase = 2.0 * Math.PI * ((elapsed % SCAN_PERIOD_MS) / (double) SCAN_PERIOD_MS);
        int offset = (int) Math.round(maxAmp * Math.sin(phase));
        return clampPosition(centerTicks + offset);

    }

    private void drivePositionToTicks(int desiredTicks, double feedforwardDegPerSec) {
        int desiredSlewed = slewTowards(targetTicksSetpoint, desiredTicks);
        targetTicksSetpoint = desiredSlewed;

        int errorTicks = desiredSlewed - reportedTicks;
        double errorDeg = errorTicks * degPerTick;

        long now = System.nanoTime();
        double dt = Math.max(1e-3, (now - lastVelNs) / 1e9);
        int deltaTicks = reportedTicks - lastVelTicks;
        double rawVelDps = (deltaTicks * degPerTick) / dt;
        velLP.update(rawVelDps, 0.0);
        double velDps = velLP.getState();
        lastVelNs = now;
        lastVelTicks = reportedTicks;

        posPid.updateError(errorDeg);
        posPid.updateFeedForwardInput(feedforwardDegPerSec);
        double pPlusF = posPid.run();

        double damp = -KD_MEAS * velDps;

        if (I_GAIN != 0.0) {
            double predicted = pPlusF + damp;
            if (Math.abs(predicted) < YAW_MAX_POWER * 0.95) {
                iAccum += errorDeg * dt;
            }
            double iTerm = clamp(I_GAIN * iAccum, -I_MAX_POWER, I_MAX_POWER);
            pPlusF += iTerm;
        }

        double cmd = pPlusF + damp;

        if (Math.abs(cmd) > 1e-4) cmd += Math.copySign(YAW_KS, cmd);

        lastCmdPower = clamp(cmd, -YAW_MAX_POWER, YAW_MAX_POWER);
        motor.setPower(lastCmdPower);
        lastErrorDeg = errorDeg;

    }

    private int slewTowards(int current, int desired) {
        long now = System.nanoTime();
        double dt = Math.max(1e-3, (now - lastLoopNs) / 1e9);
        lastLoopNs = now;

        int maxStep = (int) Math.max(1, Math.round(MAX_SLEW_TICKS_PER_SEC * dt));
        int delta = desired - current;
        if (Math.abs(delta) <= maxStep) return desired;
        return current + (delta > 0 ? maxStep : -maxStep);

    }

    private int enforceSoftLimitVector(int desired, int current) {
        if ((desired >= MAX_POSITION - SOFT_LIMIT_MARGIN && desired > current) ||
                (desired <= MIN_POSITION + SOFT_LIMIT_MARGIN && desired < current)) {
            return desired > current ? (MAX_POSITION - SOFT_LIMIT_MARGIN) : (MIN_POSITION + SOFT_LIMIT_MARGIN);
        }
        return desired;
    }

    private int wrapToNearestTicks(double desiredRelRad, int currentTicks) {
        double desiredDeg = Math.toDegrees(desiredRelRad);
        int baseTicks = (int) Math.round(desiredDeg * ticksPerDeg);
        int periodTicks = (int) Math.round(360.0 * ticksPerDeg);
        int best = baseTicks;
        int bestErr = Math.abs(baseTicks - currentTicks);
        int t1 = baseTicks + periodTicks;
        int t2 = baseTicks - periodTicks;
        if (Math.abs(t1 - currentTicks) < bestErr) {
            bestErr = Math.abs(t1 - currentTicks);
            best = t1;
        }
        if (Math.abs(t2 - currentTicks) < bestErr) {
            bestErr = Math.abs(t2 - currentTicks);
            best = t2;
        }
        return clampPosition(best);
    }

    private void addTelemetry() {
        if (!TELEMETRY_ENABLED) return;
        long now = System.currentTimeMillis();
        tele.addLine("=== SHOOTER YAW (PDm + FF) ===")
                .addData("Mode", mode::name)
                .addData("Ctrl", control::name)
                .addData("AllianceRed", "%b", isRed)
                .addData("TargetId", "%d", resolveTargetId())
                .addData("CurTicks", "%d", reportedTicks)
                .addData("DesiredTicks", "%d", lastDesiredTicks)
                .addData("SetptTicks", "%d", targetTicksSetpoint)
                .addData("Err(deg)", "%.2f", lastErrorDeg)
                .addData("CmdPower", "%.2f", lastCmdPower)
                .addData("LL Age(ms)", "%d", now - lastLLFreshMs)
                .addData("Omega(dps)", "%.1f", Math.toDegrees(lastRobotOmegaRad));
        if (control == ControlState.SEEK_PATTERN) {
            tele.addData("SeekIdx", "%d", patternIdx)
                    .addData("ChosenId", "%d", chosenPatternId)
                    .addData("SeekElapsed(ms)", "%d", now - sweepStartMs);
        }
    }

    private int getCurrentTicks() {
        return motor.getCurrentPosition();
    }

    private int clampPosition(int pos) {
        return (int) clamp(pos, MIN_POSITION, MAX_POSITION);
    }

    private int resolveTargetId() {
        return (targetId > 0) ? targetId : (isRed ? APRIL_TAG_RED :
                APRIL_TAG_BLUE);
    }

    private double getRobotYawRad() {
        try {
            return (follower != null && follower.getPose() != null) ?
                    follower.getPose().getHeading() : 0.0;
        } catch (Throwable t) {
            return 0.0;
        }
    }

    private double getRobotOmegaRad() {
        try {
            return follower != null ? follower.getAngularVelocity() :
                    0.0;
        } catch (Throwable t) {
            return 0.0;
        }
    }

    private double getGimbalRelativeRad() {
        int ticks = reportedTicks - CENTER_POS;
        return
                Math.toRadians(ticks * degPerTick);
    }

    private static double normalizeDeg(double deg) {
        double x = deg % 360.0;
        if (x > 180.0) x -= 360.0;
        if (x <= -180.0) x += 360.0;
        return x;
    }

    private static double normalizeSignedRad(double rad) {
        double a = MathFunctions.normalizeAngle(rad);
        if (a > Math.PI) a -= 2.0 * Math.PI;
        return a;
    }

    private static double blendAngles(double a, double b, double alpha) {
        double diff =
                normalizeSignedRad(b - a);
        return normalizeSignedRad(a + alpha * diff);
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
}