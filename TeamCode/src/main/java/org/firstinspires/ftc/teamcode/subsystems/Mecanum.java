package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.REQUIRED_STABLE_FRAMES;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.ROT_DB;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.GlobalConfig.SLOW_MODE_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingController;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Mecanum {
    public static class AutoDriveCommand {
        public double forward = 0;
        public double strafe = 0;
        public double turn = 0;
        public boolean fieldCentric = true;
        public double offsetHeading = 0;
    }

    private boolean headingLockEnabled = false;
    private boolean headingLockActive = false;
    private double lockHeadingDeg = 0.0;
    private int stableFrames = 0;

    private final HeadingController teleopHeadingCtrl = new HeadingController();

    private final OpMode opmode;
    private final TelemetryHelper tele;

    private Follower follower;
    private final GamepadMap map;

    private SubsystemMode mode = SubsystemMode.MANUAL;
    private boolean slowMode = false;
    private boolean fieldCentricEnabled = false;

    private final AutoDriveCommand autoCmd = new AutoDriveCommand();

    public Mecanum(OpMode opmode, GamepadMap map) {
        this.opmode = opmode;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void init() {
        follower = createFollower(opmode.hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
        follower.startTeleopDrive();
    }

    public void startAuto() {
        mode = SubsystemMode.AUTO;
        follower.startTeleopDrive();
        clearAutoCommand();
    }

    public void setAutoDrive(double forward, double strafe, double turn, boolean fieldCentric, double offsetHeading) {
        autoCmd.forward = forward;
        autoCmd.strafe = strafe;
        autoCmd.turn = turn;
        autoCmd.fieldCentric = fieldCentric;
        autoCmd.offsetHeading = offsetHeading;
    }

    public void clearAutoCommand() {
        setAutoDrive(0, 0, 0, true, 0);
    }

    public void operate() {
        if (mode == SubsystemMode.MANUAL) {
            handleTeleopInputs();
        } else {
            applyAutoCommand();
        }
        follower.update();

        addTelemetry();
    }

    public void enableTeleopHeadingLock(boolean enable) {
        headingLockEnabled = enable;
        if (!enable) headingLockActive = false;
    }

    private void toggleHeadingLock() {
        headingLockEnabled = !headingLockEnabled;
        if (!headingLockEnabled) headingLockActive = false;
    }

    private void handleTeleopInputs() {
        if (map == null) return;

        handleToggles();

        double rawTurn = slowMode ? -map.rotate * SLOW_MODE_MULTIPLIER : -map.rotate;
        double currentHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        double turnCmd;

        if (headingLockEnabled) {
            boolean stickIdle = Math.abs(rawTurn) <= ROT_DB;
            double omega = follower.getPoseTracker().getAngularVelocity(); // rad/s
            double omegaDeg = Math.toDegrees(omega);
            boolean spinIdle = Math.abs(omegaDeg) <= 5.0;

            if (!stickIdle) {
                headingLockActive = false;
                stableFrames = 0;
                turnCmd = rawTurn;
            } else {
                if (!headingLockActive) {
                    if (spinIdle) stableFrames++;
                    else stableFrames = 0;
                    if (stableFrames >= REQUIRED_STABLE_FRAMES) {
                        lockHeadingDeg = currentHeadingDeg;
                        teleopHeadingCtrl.reset();
                        headingLockActive = true;
                    }
                    turnCmd = 0.0;
                } else {
                    turnCmd = teleopHeadingCtrl.update(lockHeadingDeg, currentHeadingDeg);
                }
            }
        } else {
            headingLockActive = false;
            stableFrames = 0;
            turnCmd = rawTurn;
        }

        double forward = slowMode ? -map.forward * SLOW_MODE_MULTIPLIER : -map.forward;
        double strafe = slowMode ? -map.strafe * SLOW_MODE_MULTIPLIER : -map.strafe;

        follower.setTeleOpDrive(forward, strafe, turnCmd, !fieldCentricEnabled);
    }

    private void applyAutoCommand() {
        follower.setTeleOpDrive(
                -autoCmd.forward,
                -autoCmd.strafe,
                autoCmd.turn,
                !autoCmd.fieldCentric,
                autoCmd.offsetHeading
        );
    }

    private void handleToggles() {
        if (map.slowModeToggle) slowMode = !slowMode;
        if (map.fieldCentricToggle) fieldCentricEnabled = !fieldCentricEnabled;
        if (map.angleLockToggle) toggleHeadingLock();
        follower.setMaxPowerScaling(slowMode ? SLOW_MODE_MULTIPLIER : 1.0);
    }

    private void addTelemetry() {
        Pose p = follower.getPose();
        Vector v = follower.getVelocity();
        tele.addLine("--- Mecanum ---")
                .addData("Mode", mode::name)
                .addData("Pose", "(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()))
                .addData("Vel", "(%.1f, %.1f)", v.getXComponent(), v.getYComponent())
                .addData("SlowMode", "%b", slowMode)
                .addData("FieldCentric", "%b", fieldCentricEnabled)
                .addData("HeadingLockEnabled", "%b", headingLockEnabled)
                .addData("HeadingLockActive", "%b", headingLockActive)
                .addData("LockHeadingDeg", "%.1f", lockHeadingDeg);
    }

    public Follower getFollower() {
        return follower;
    }

    public void setStartingPose(Pose p) {
        follower.setStartingPose(p);
        follower.update();
    }
}