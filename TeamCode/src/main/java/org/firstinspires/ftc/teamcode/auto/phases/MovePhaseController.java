package org.firstinspires.ftc.teamcode.auto.phases;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.LEAVE_LAUNCH_LINE_BLUE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.LEAVE_LAUNCH_LINE_RED;
import static org.firstinspires.ftc.teamcode.config.AutoMotionConfig.DEFAULT_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoMotionConfig.DRIVE_STOP_DIST_IN;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.auto.geom.FieldBounds;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class MovePhaseController {
    public enum Result {CONTINUE, TO_DEPOSIT, TO_DONE}

    private final MotionController motion;
    private final HeadingTarget heading;
    private final boolean isRed;
    private final boolean move;
    private final boolean shootPreloaded;
    private final boolean activelyIntake;
    private final TelemetryHelper tele;

    private boolean initialized = false;
    private Pose target;

    public MovePhaseController(MotionController motion,
                               HeadingTarget heading,
                               boolean isRed,
                               boolean isAudienceSide,
                               boolean move,
                               boolean shootPreloaded,
                               boolean activelyIntake,
                               TelemetryHelper tele) {
        this.motion = motion;
        this.heading = heading;
        this.isRed = isRed;
        this.move = move;
        this.shootPreloaded = shootPreloaded;
        this.activelyIntake = activelyIntake;
        this.tele = tele;
    }

    public Result update(Pose currentPose) {
        if (!move) {
            return shootPreloaded ? Result.TO_DEPOSIT : Result.TO_DONE;
        }

        // If shoot preloaded: go deposit
        if (shootPreloaded) return Result.TO_DEPOSIT;

        // If move enabled and no shoot preloaded and no active intake => use LEAVE_LAUNCH_LINE pose; then DONE
        if (!activelyIntake) {
            if (!initialized) {
                // Leave launch line
                target = isRed ? LEAVE_LAUNCH_LINE_RED : LEAVE_LAUNCH_LINE_BLUE;
                target = FieldBounds.clampToAllianceRect(target, isRed);
                initialized = true;
            }
            boolean done = motion.driveToPose(target, DRIVE_STOP_DIST_IN, DEFAULT_TIMEOUT_S, heading);
            addTelemetry();
            return done ? Result.TO_DONE : Result.CONTINUE;
        }

        if (!initialized) {
            double y = currentPose.getY() + 18.0;
            target = new Pose(currentPose.getX(), y, currentPose.getHeading());
            target = FieldBounds.clampToAllianceRect(target, isRed);
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
