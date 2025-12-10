package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@Autonomous(name = "Pedro Pose Test", group = "Tuning")
@Configurable
public class PedroPoseTest extends LinearOpMode {

    // Edit these in Panels to dynamically change target
    public static double TARGET_X = 72.0;
    public static double TARGET_Y = 72.0;
    public static double TARGET_HEADING_DEG = 0.0;

    // Starting pose
    public static double START_X = 72.0;
    public static double START_Y = 72.0;
    public static double START_HEADING_DEG = 0.0;

    // Control
    public static boolean RESET_TO_START = false;  // Toggle in Panels to reset position
    public static double REPATH_THRESHOLD = 2.0;   // Re-path if target changes by this much

    // Last known target (to detect changes)
    private double lastX = TARGET_X;
    private double lastY = TARGET_Y;
    private double lastHeadingDeg = TARGET_HEADING_DEG;

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadMap map = new GamepadMap(this);
        Mecanum drive = new Mecanum(this, map);
        drive.init();

        Follower follower = drive.getFollower();
        TelemetryHelper tele = new TelemetryHelper(this, true);

        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        drive.setStartingPose(startPose);

        telemetry.addLine("Pedro Pose Test Ready");
        telemetry.addLine("Edit TARGET_X, TARGET_Y, TARGET_HEADING_DEG in Panels");
        telemetry.addLine("Press Start to begin");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        drive.startAuto();

        // Initial path
        pathToTarget(follower);

        while (opModeIsActive()) {
            // Check for reset toggle
            if (RESET_TO_START) {
                RESET_TO_START = false;
                Pose resetPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
                follower.setStartingPose(resetPose);
                follower.update();
                pathToTarget(follower);
            }

            // Check if target changed significantly
            if (targetChanged()) {
                pathToTarget(follower);
                updateLastTarget();
            }

            // Re-path when current path completes
            if (!follower.isBusy()) {
                pathToTarget(follower);
            }

            follower.update();

            // Telemetry
            Pose current = follower.getPose();
            Pose target = getTargetPose();
            double distToTarget = Math.hypot(target.getX() - current.getX(), target.getY() - current.getY());
            double headingError = Math.toDegrees(target.getHeading() - current.getHeading());

            tele.addLine("=== PEDRO POSE TEST ===")
                    .addData("Current", "(%.1f, %.1f, %.1f°)", current.getX(), current.getY(), Math.toDegrees(current.getHeading()))
                    .addData("Target", "(%.1f, %.1f, %.1f°)", TARGET_X, TARGET_Y, TARGET_HEADING_DEG)
                    .addData("Distance", "%.2f in", distToTarget)
                    .addData("Heading Err", "%.1f°", headingError)
                    .addData("Busy", "%b", follower.isBusy());

            TelemetryHelper.update();
            sleep(20);
        }
    }

    private Pose getTargetPose() {
        return new Pose(TARGET_X, TARGET_Y, Math.toRadians(TARGET_HEADING_DEG));
    }

    private void pathToTarget(Follower follower) {
        Pose target = getTargetPose();
        Pose current = follower.getPose();

        // Simple bezier curve with midpoint control
        double midX = (current.getX() + target.getX()) / 2.0;
        double midY = (current.getY() + target.getY()) / 2.0;
        Pose control = new Pose(midX, midY, target.getHeading());

        follower.followPath(
                follower.pathBuilder()
                        .addPath(new com.pedropathing.geometry.BezierCurve(
                               current,
                                control,
                                target))
                        .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                        .build()
        );
    }

    private boolean targetChanged() {
        return Math.abs(TARGET_X - lastX) > REPATH_THRESHOLD ||
                Math.abs(TARGET_Y - lastY) > REPATH_THRESHOLD ||
                Math.abs(TARGET_HEADING_DEG - lastHeadingDeg) > 5.0;
    }

    private void updateLastTarget() {
        lastX = TARGET_X;
        lastY = TARGET_Y;
        lastHeadingDeg = TARGET_HEADING_DEG;
    }
}