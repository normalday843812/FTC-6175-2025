package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.GlobalConfig.SLOW_MODE_MULTIPLIER;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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

    private final OpMode opmode;
    private final TelemetryHelper tele;

    private Follower follower;
    private final GamepadMap map;

    private SubsystemMode mode = SubsystemMode.MANUAL;
    private boolean slowMode = false;
    private boolean fieldCentricEnabled = true;

    private final AutoDriveCommand autoCmd = new AutoDriveCommand();

    public Mecanum(OpMode opmode, GamepadMap map) {
        this.opmode = opmode;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void init() {
        follower = Constants.createFollower(opmode.hardwareMap);
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
        follower.update();

        if (mode == SubsystemMode.MANUAL) {
            handleTeleopInputs();
        } else {
            applyAutoCommand();
        }

        addTelemetry();
    }

    private void handleTeleopInputs() {
        if (map == null) return;

        if (map.slowModeToggle) slowMode = !slowMode;
        if (map.fieldCentricToggle) fieldCentricEnabled = !fieldCentricEnabled;

        double f = slowMode ? -map.forward * SLOW_MODE_MULTIPLIER : -map.forward;
        double s = slowMode ? -map.strafe * SLOW_MODE_MULTIPLIER : -map.strafe;
        double t = slowMode ? -map.rotate * SLOW_MODE_MULTIPLIER : -map.rotate;

        follower.setTeleOpDrive(f, s, t, fieldCentricEnabled);
    }

    private void applyAutoCommand() {
        follower.setTeleOpDrive(
                -autoCmd.forward,
                -autoCmd.strafe,
                autoCmd.turn,
                autoCmd.fieldCentric,
                autoCmd.offsetHeading
        );
    }

    private void handleToggles() {
        if (map.slowModeToggle) slowMode = !slowMode;
        if (map.fieldCentricToggle) fieldCentricEnabled = !fieldCentricEnabled;
    }

    private void addTelemetry() {
        Pose p = follower.getPose();
        Vector v = follower.getVelocity();
        tele.addLine("--- Mecanum ---")
                .addData("Mode", mode::name)
                .addData("Pose", "(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()))
                .addData("Velocity", "(%.1f, %.1f)", v.getXComponent(), v.getYComponent());
    }

    public Follower getFollower() {
        return follower;
    }

    public void setStartingPose(Pose p) {
        follower.setStartingPose(p);
        follower.update();
    }
}