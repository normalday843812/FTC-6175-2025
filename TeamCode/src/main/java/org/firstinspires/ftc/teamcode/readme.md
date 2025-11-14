Reasoning
Empty reasoning item
assistant
You’re running into classic “derivative on error” kick + quantization + friction problems. With the
new fast loop, the setpoint moves when the robot spins (world lock), so d(error)/dt includes d(
setpoint)/dt. That makes D blow up at high robot angular velocity. Cranking D just below the
“explosion” threshold can leave a slow drift, so you try I, but I then fights friction and adds
low‑freq oscillations. This is why you’re seeing “either oscillates or sluggish.”

What fixes it

Use derivative on measurement (velocity damping), not derivative of error. D should act on the
gimbal’s measured angular rate, not on error. This removes “derivative kick.”
Keep I at 0 (or tiny) and clamp it; let static friction compensation (Ks) and a correct
P+velocity-damping do the work.
Keep LL as the world-target setter (like you have now) and keep your robot-omega feedforward (F).
Those are good.
Below is a drop-in update that:

Leaves your RUN_WITHOUT_ENCODER + world-target architecture intact
Disables D in the FilteredPIDFController (set it to 0) and adds a separate “derivative on
measurement” damping term
Adds a small optional external integrator with anti-windup clamp (default off)
Low-passes the measured velocity to avoid noise
Keeps friction compensation (Ks) and feedforward (F)
How to tune with this

Set I_GAIN = 0 for now.
Tune P first. Increase until you get a little overshoot/oscillation, then back off 10–20%.
Increase KD_MEAS until oscillation goes away and steps settle crisply. KD_MEAS is “power per deg/s.”
Tune F (feedforward) so when the robot spins, the gimbal almost keeps up without needing big P
corrections.
Set YAW_KS so tiny steps don’t stick.
Only if you still see a tiny steady offset in steady-state, add a very small I_GAIN and clamp it via
I_MAX_POWER (0.05–0.2 typical).
Updated code

ShooterYawConfig.java (add new gains and set PIDF D to 0)

package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;

@Configurable
public class ShooterYawConfig {
public static boolean TELEMETRY_ENABLED = true;

public static int MIN_POSITION = -340;
public static int MAX_POSITION = 450;
public static int CENTER_POS = 0;
public static double TICKS_PER_DEG_INIT = 3.0;

public static double CONTROL_LEVEL = 20;

public static double AUTO_LOCK_DEADBAND_DEG = 1.5;

// RUN_WITHOUT_ENCODER
public static double YAW_MAX_POWER = 1.0; // power cap while tuning
public static double YAW_KS = 0.02; // static friction compensation

public static long LL_GRACE_MS = 150;
public static long REACQUIRE_AFTER_MS = 300;

// Skip LL world-target updates when robot spinning very fast
public static double OMEGA_LL_UPDATE_MAX_RAD = 3.0; // ~172 deg/s

// Blend factor for world target updates
public static double TARGET_BLEND_ALPHA = 0.6;

// Scan/search
public static boolean REACQUIRE_SCAN_ENABLED = true;
public static int MIN_SCAN_AMPLITUDE_TICKS = 4;
public static double SCAN_BAND_DEG = 12.0;
public static long SCAN_PERIOD_MS = 1200;
public static long SEEK_WINDOW_MS = 1200;
public static long STABLE_HOLD_MS_DEFAULT = 350;

public static int SOFT_LIMIT_MARGIN = 2;

public static double METERS_PER_POSE_UNIT = 0.0254;

public static double MAX_SLEW_TICKS_PER_SEC = 3000.0;

// FilteredPIDF used only for P and F now (D=0, I=0 by default)
// Units: P [power/deg], F [power/(deg/s)]
public static FilteredPIDFCoefficients PIDF_COEFFICIENTS =
new FilteredPIDFCoefficients(
0.012, // P
0.0, // I (unused; external I recommended if needed)
0.0, // D off (we use derivative on measurement externally)
0.20, // T (derivative LPF param not used now, leave ~0.2)
0.0025 // F (robot omega -> power)
);

// Derivative on measurement damping (measured gimbal rate)
public static double KD_MEAS = 0.004; // power per (deg/s)
public static double VEL_LP_ALPHA = 0.2; // low-pass for measured velocity (0<alpha<1)

// Optional external integral (disabled by default)
public static double I_GAIN = 0.0; // power per (deg*s)
public static double I_MAX_POWER = 0.15; // clamp magnitude of integral contribution
}

ShooterYaw.java (updates marked in comments)

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

// Keep FilteredPIDF for P and F only
private final FilteredPIDFController posPid = new FilteredPIDFController(PIDF_COEFFICIENTS);

// Measured-velocity damping (derivative on measurement)
private final LowPassFilter velLP = new LowPassFilter(VEL_LP_ALPHA);
private long lastVelNs = System.nanoTime();
private int lastVelTicks = 0;

// Optional external integral with clamp
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
private int targetTicksSetpoint = CENTER_POS;

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
    // Ensure D=0 for FilteredPIDF (we do derivative on measurement externally)
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

public boolean isPatternChosen() { return chosenPatternId >= 0; }
public int getChosenPatternId() { return chosenPatternId; }
public boolean isLockedOnTarget() { return mode == SubsystemMode.AUTO && control ==
ControlState.AUTO_WORLD_LOCK && worldTargetInitialized; }

public void center() { targetTicksSetpoint = CENTER_POS; }

public void lockAllianceGoal() {
mode = SubsystemMode.AUTO;
control = ControlState.AUTO_WORLD_LOCK;
targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
posPid.reset(); posPid.setD(0.0);
worldTargetInitialized = false;
lastLLFreshMs = 0L;
iAccum = 0.0;
}

public void resetShooterYaw() {
motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
reportedTicks = 0;
targetTicksSetpoint = CENTER_POS;
posPid.reset(); posPid.setD(0.0);
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

    double errorDeg = (desiredTicks - reportedTicks) * degPerTick;
    lastErrorDeg = errorDeg;

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
posPid.reset(); posPid.setD(0.0);
worldTargetInitialized = false;
lastLLFreshMs = 0L;
iAccum = 0.0;
}

private void disableAutoToManual() {
mode = SubsystemMode.MANUAL;
control = ControlState.MANUAL_HOLD;
posPid.reset(); posPid.setD(0.0);
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

// Core: position control to power with derivative on measurement + FF + Ks (+ optional I with
clamp)
private void drivePositionToTicks(int desiredTicks, double feedforwardDegPerSec) {
// Slew desired setpoint to avoid big leaps
int desiredSlewed = slewTowards(targetTicksSetpoint, desiredTicks);
targetTicksSetpoint = desiredSlewed;

    // Error in degrees
    int errorTicks = desiredSlewed - reportedTicks;
    double errorDeg = errorTicks * degPerTick;

    // --- Measured velocity (deg/s), low-passed ---
    long now = System.nanoTime();
    double dt = Math.max(1e-3, (now - lastVelNs) / 1e9);
    int deltaTicks = reportedTicks - lastVelTicks;
    double rawVelDps = (deltaTicks * degPerTick) / dt;
    velLP.update(rawVelDps, 0.0);
    double velDps = velLP.getState();
    lastVelNs = now;
    lastVelTicks = reportedTicks;

    // --- P + F from FilteredPIDFController (D=0, I=0) ---
    posPid.updateError(errorDeg);
    posPid.updateFeedForwardInput(feedforwardDegPerSec); // converts deg/s -> power
    double pPlusF = posPid.run();

    // --- Derivative on measurement (velocity damping) ---
    double damp = -KD_MEAS * velDps;

    // --- Optional external integral with clamp (off by default) ---
    if (I_GAIN != 0.0) {
        // simple anti-windup: integrate only when not saturated by a big margin
        double predicted = pPlusF + damp;
        if (Math.abs(predicted) < YAW_MAX_POWER * 0.95) {
            iAccum += errorDeg * dt; // deg*s
        }
        double iTerm = clamp(I_GAIN * iAccum, -I_MAX_POWER, I_MAX_POWER);
        pPlusF += iTerm;
    }

    double cmd = pPlusF + damp;

    // Static friction compensation
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
if (Math.abs(t1 - currentTicks) < bestErr) { bestErr = Math.abs(t1 - currentTicks); best = t1; }
if (Math.abs(t2 - currentTicks) < bestErr) { bestErr = Math.abs(t2 - currentTicks); best = t2; }
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

private int getCurrentTicks() { return motor.getCurrentPosition(); }
private int clampPosition(int pos) { return (int) clamp(pos, MIN_POSITION, MAX_POSITION); }
private int resolveTargetId() { return (targetId > 0) ? targetId : (isRed ? APRIL_TAG_RED :
APRIL_TAG_BLUE); }
private double getRobotYawRad() { try { return (follower != null && follower.getPose() != null) ?
follower.getPose().getHeading() : 0.0; } catch (Throwable t) { return 0.0; } }
private double getRobotOmegaRad() { try { return follower != null ? follower.getAngularVelocity() :
0.0; } catch (Throwable t) { return 0.0; } }
private double getGimbalRelativeRad() { int ticks = reportedTicks - CENTER_POS; return
Math.toRadians(ticks * degPerTick); }

private static double normalizeDeg(double deg) { double x = deg % 360.0; if (x > 180.0) x -= 360.0;
if (x <= -180.0) x += 360.0; return x; }
private static double normalizeSignedRad(double rad) { double a = MathFunctions.normalizeAngle(rad);
if (a > Math.PI) a -= 2.0 * Math.PI; return a; }
private static double blendAngles(double a, double b, double alpha) { double diff =
normalizeSignedRad(b - a); return normalizeSignedRad(a + alpha * diff); }

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

Why this removes the oscillations

D now damps the measured gimbal rate, so it doesn’t spike when the setpoint moves (no derivative
kick).
The P gain can be set high enough for snappy response; KD_MEAS soaks up oscillation energy.
Ks helps overcome static friction so you don’t “need” I to move off deadband.
If the mechanism still needs it, the small external I has anti-windup so it won’t blow up.
Quick tuning checklist

Set PIDF_COEFFICIENTS.D = 0 (already in the config).
Start: P=0.01–0.02, KD_MEAS=0.003–0.006, F=0.002–0.004, Ks=0.01–0.03.
Verify response to a step (command a 20–40° jump). If underdamped, raise KD_MEAS slightly; if
overdamped/slow, raise P slightly. Keep I_GAIN=0 at first.
Spin the robot. If world-lock lags, increase F a bit.
Only add a small I later if you see a consistent offset at rest; clamp with I_MAX_POWER.
This should stop the “it oscillates unless I make P tiny” problem and give you the “spin-fast but
rock-steady gimbal” you want.