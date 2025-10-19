package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.GlobalConfig.SLOW_MODE_MULTIPLIER;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Mecanum {
    private Follower follower;
    public static Pose startingPose;
    private boolean slowMode = false;
    private boolean fieldCentricEnabled = true;
    private final OpMode opmode;
    private final GamepadMap map;
    private final TelemetryHelper tele;

    public Mecanum(OpMode opmode, GamepadMap map) {
        this.opmode = opmode;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void init() {
        follower = Constants.createFollower(opmode.hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
    }

    public void start() {
        follower.startTeleopDrive();
    }
    public void operate() {
        follower.update();

        handleToggles();

        if (!slowMode) {
            follower.setTeleOpDrive(
                    -map.forward,
                    -map.strafe,
                    -map.rotate,
                    fieldCentricEnabled
            );
        } else  {
            follower.setTeleOpDrive(
                    -map.forward * SLOW_MODE_MULTIPLIER,
                    -map.strafe * SLOW_MODE_MULTIPLIER,
                    -map.rotate * SLOW_MODE_MULTIPLIER,
                    fieldCentricEnabled
            );
        }

        addTelemetry();
    }

    private void handleToggles() {
        if (map.slowModeToggle) slowMode = !slowMode;
        if (map.fieldCentricToggle) fieldCentricEnabled = !fieldCentricEnabled;
    }

    private void addTelemetry() {
        Pose pose = follower.getPose();
        Vector velocity = follower.getVelocity();
        double angularVelocityDeg = Math.toDegrees(follower.getAngularVelocity());
        double headingDeg = Math.toDegrees(pose.getHeading());

        tele.addLine("--- Mecanum ---")
                .addData("Pose", "(%.1f, %.1f) | %.1f°", pose.getX(), pose.getY(), headingDeg)
                .addData("Velocity", "(%.1f, %.1f) | %.1f°", velocity.getXComponent(), velocity.getYComponent(), angularVelocityDeg)
                .addData("Slow Mode", "%b%n", slowMode);
    }
}