package org.firstinspires.ftc.teamcode.auto.phases;

import static org.firstinspires.ftc.teamcode.config.AutoMotionConfig.DEFAULT_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoMotionConfig.DRIVE_STOP_DIST_IN;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.auto.motion.HeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class MovePhaseController {
    public enum Result {CONTINUE, TO_DEPOSIT, TO_DONE}

    private final MotionController motion;
    private final HeadingTarget heading;
    private final boolean move;
    private final boolean shootPreloaded;
    private final TelemetryHelper tele;

    private boolean initialized = false;
    private Pose target;

    public MovePhaseController(MotionController motion,
                               HeadingTarget heading,
                               boolean isRed,
                               boolean isAudienceSide,
                               boolean move,
                               boolean shootPreloaded,
                               TelemetryHelper tele) {
        this.motion = motion;
        this.heading = heading;
        this.move = move;
        this.shootPreloaded = shootPreloaded;
        this.tele = tele;
    }

    public Result update(Pose currentPose) {
        if (!move) {
            return shootPreloaded ? Result.TO_DEPOSIT : Result.TO_DONE;
        }

        if (shootPreloaded) {
            return Result.TO_DEPOSIT;
        }

        if (!initialized) {
            // Leave launch line
            double y = currentPose.getY() + 18.0;
            target = new Pose(currentPose.getX(), y, currentPose.getHeading());
            initialized = true;
        }

        boolean done = motion.driveToPose(target, DRIVE_STOP_DIST_IN, DEFAULT_TIMEOUT_S, heading);
        addTelemetry();
        return done ? Result.TO_DONE : Result.CONTINUE;
    }

    private void addTelemetry() {
        tele.addLine("--- MOVE ---")
                .addData("Target", "(%.1f, %.1f)", target.getX(), target.getY());
    }
}