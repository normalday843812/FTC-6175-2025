package org.firstinspires.ftc.teamcode.auto.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.motion.HeadingController;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

/**
 * Locks heading to a fixed point 10” in front of start pose while driving a triangular pattern.
 * Pattern: left half-length, then down-right, then up-right, then all the way left, repeat.
 *
 * High level: we continuously compute the angle to a target point (relative to the robot pose),
 * then feed vx, vy translation commands that trace a triangle. Heading is decoupled from translation
 * through the HeadingTarget interface. Simple, readable, and great for tuning.
 */
@Autonomous(name = "Test: Heading Lock Point Triangle", group = "Test")
public class TestHeadingLockPointTriangle extends LinearOpMode {

    // Made by ChatGPT, may not work.

    /**
     * Minimal HeadingTarget that aims the robot toward a fixed (x, y) world point.
     * The math is classic atan2(y, x). The normalization keeps the return in [0, 360).
     *
     * Note: units follow your Pose convention. Here we assume inches for x/y and degrees for heading.
     */
    private static class FacingPointHeadingTarget implements HeadingTarget {
        // Target point in field coordinates (immutable once constructed)
        private final double tx, ty;

        FacingPointHeadingTarget(double x, double y) {
            this.tx = x;
            this.ty = y;
        }

        @Override
        public double getTargetHeadingDeg(Pose p) {
            // Vector from robot to target
            double dy = ty - p.getY();
            double dx = tx - p.getX();
            // Convert atan2 to degrees for the HeadingController
            double yaw = Math.toDegrees(Math.atan2(dy, dx));
            return normalize0to360(yaw);
        }

        @Override
        public String debugName() {
            return "FacingPoint";
        }

        /**
         * Wrap any angle into [0, 360). This avoids negative headings and >360 overflow.
         * Tiny helper, easy to test, keeps downstream logic simpler.
         */
        private static double normalize0to360(double a) {
            while (a < 0) a += 360;
            while (a >= 360) a -= 360;
            return a;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Bring up the drivetrain. Passing 'null' for hardware map extensions is intentional in this test harness.
        Mecanum drive = new Mecanum(this, null);
        drive.init();

        // Define the starting field pose: x=72, y=72, heading=90 degrees (pointing "up" in field frame).
        Pose start = new Pose(72, 72, Math.toRadians(90));
        drive.setStartingPose(start);
        drive.startAuto();

        // Create controllers. HeadingController handles yaw to a target; MotionController mixes translation with heading logic.
        HeadingController hc = new HeadingController();
        MotionController mc = new MotionController(drive, hc, new TelemetryHelper(this, true));

        // Pick the fixed look-at point: 10 inches in front of start pose along +Y from that pose.
        // This gives an intuitive "face forward while sliding around" behavior.
        double targetX = start.getX();
        double targetY = start.getY() + 10.0;
        HeadingTarget pointLock = new FacingPointHeadingTarget(targetX, targetY);

        // Standard opmode start gate
        if (isStopRequested()) return;
        waitForStart();

        // Time-based segmenting for the triangular path. Each segment = 2 seconds, cycle length = 8 seconds.
        long t0 = System.currentTimeMillis();
        while (opModeIsActive()) {
            double t = (System.currentTimeMillis() - t0) / 1000.0;

            // vx and vy are field-centric translation commands as expected by MotionController.translateFacing
            // Magnitudes chosen to be safe indoors and easily observable. Adjust as needed during tuning.
            double vx, vy;
            double seg = Math.floor((t % 8.0) / 2.0);

            // Segment map:
            // 0: move left
            // 1: move down-right
            // 2: move up-right
            // 3: move left again (long leg), then repeat
            if (seg == 0) {
                vx = -0.4;
                vy = 0.0;
            } else if (seg == 1) {
                vx = 0.35;
                vy = -0.35;
            } else if (seg == 2) {
                vx = 0.35;
                vy = 0.35;
            } else {
                vx = -0.4;
                vy = 0.0;
            }

            // Drive translation while the HeadingController keeps the robot facing the fixed point.
            // This isolates heading behavior for clean tuning and observation.
            mc.translateFacing(vx, vy, pointLock);

            // Push one iteration of the subsystem loop, flush telemetry, and keep the control loop modestly paced.
            drive.operate();
            TelemetryHelper.update();
            sleep(20); // ~50 Hz loop; adjust for your HW if needed
        }
    }
}
