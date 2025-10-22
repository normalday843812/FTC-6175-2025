package org.firstinspires.ftc.teamcode.auto.testing;

import android.annotation.SuppressLint;
import android.graphics.Color;
import android.util.Size;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.motion.FixedFieldHeading;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingController;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.auto.vision.BlobDetector;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Intake.AutoMode;
import org.firstinspires.ftc.teamcode.subsystems.IntakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.Menu;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

/**
 * Test opmode: chase visible purple/green game pieces using a color-blob detector
 * while facing a fixed field heading (half-field facing).
 *
 * Controls:
 * - Before start, pick Alliance (Red/Blue) and whether movement is enabled.
 * - During run, press X to toggle chase/move on/off at runtime (simple debounce included).
 *
 * Behavior:
 * - Uses webcam and two color blob processors (purple and green) and selects the largest.
 * - Generates field-centric vx, vy to approach the blob while heading is controlled by FixedFieldHeading.
 * - Intake runs forward while "RUNNING"; after a color-sensed capture, the robot dwells for a short period.
 *
 * Notes:
 * - All numeric gains are conservative for easy indoor testing.
 * - This is a minimal demo harness for tuning and instrumentation; not a final autos.
 */
@Autonomous(name = "Test: Intake", group = "Test")
public class TestIntakeChase extends LinearOpMode {

    // Made by ChatGPT, may not work.

    /**
     * Simple two-state controller:
     * RUNNING -> actively intakes + moves toward target
     * CAPTURED_DWELL -> pause briefly after detecting a captured piece via color sensor
     */
    private enum TestState {RUNNING, CAPTURED_DWELL}

    // Movement parameters (approach gain and speed limits). Tune to taste.
    private static final double BLOB_APPROACH_GAIN = 0.6;
    private static final double BLOB_FORWARD_MAX = 0.6;
    private static final double BLOB_STRAFE_MAX = 0.5;

    // Dwell time after a capture; keeps the mechanism stable and lets telemetry catch up.
    private static final double CAPTURE_DWELL_S = 1.0;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        // Simple pre-start menu for alliance and motion enable. Uses gamepad buttons for selection.
        Menu menu = new Menu(this)
                .add(new Menu.Item(
                        "Alliance",
                        "Red (B)", () -> gamepad1.b,
                        "Blue (X)", () -> gamepad1.x,
                        true
                ))
                .add(new Menu.Item(
                        "Chase/Move?",
                        "Yes (A)", () -> gamepad1.a,
                        "No  (B)", () -> gamepad1.b,
                        false // DEFAULT = NO MOVEMENT
                ));

        menu.showUntilStart();

        boolean isRed = menu.get("Alliance");
        boolean moveChaseEnabled = menu.get("Chase/Move?");

        // Hardware bring-up: intake, color sensor, webcam (single camera used by VisionPortal).
        RobotHardware hw = new RobotHardware(this);
        hw.initIntake();
        hw.initIntakeColorSensor();
        hw.initWebcam();

        // Drivetrain: field pose is arbitrary for this test; facing up (90 deg) at (72,72).
        Mecanum drive = new Mecanum(this, null);
        drive.init();
        Pose start = new Pose(72, 72, Math.toRadians(90));
        drive.setStartingPose(start);
        drive.startAuto();

        // Intake and color sensor subsystems for capture detection and roller control.
        Intake intake = new Intake(hw.getIntakeMotor(), null, this);
        intake.startAuto();

        IntakeColorSensor intakeColor = new IntakeColorSensor(hw.getIntakeColorSensor(), this);

        // Motion controller: translation requests + heading target. Face 0 deg for Red, 180 deg for Blue.
        HeadingController hc = new HeadingController();
        TelemetryHelper motionTele = new TelemetryHelper(this, true);
        MotionController motion = new MotionController(drive, hc, motionTele);
        HeadingTarget faceHalf = new FixedFieldHeading(isRed ? 0 : 180, "FaceHalf");

        // Vision processors configured for specific artifact color ranges.
        // ROI is a central band; morphological operations reduce noise and close gaps.
        ColorBlobLocatorProcessor purpleProc = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.8, 0.8, 0.8, -0.7))
                .setDrawContours(false)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 255))
                .setBlurSize(5).setDilateSize(8).setErodeSize(8)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        ColorBlobLocatorProcessor greenProc = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.8, 0.8, 0.8, -0.7))
                .setDrawContours(false)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 255))
                .setBlurSize(5).setDilateSize(8).setErodeSize(8)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // VisionPortal: ties camera to processors; using a modest resolution for throughput.
        Size frameSize = new Size(320, 240);
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(purpleProc)
                .addProcessor(greenProc)
                .setCameraResolution(frameSize)
                .setCamera(hw.getWebcam1())
                .build();

        // Thin helper that chooses "best" blob from processors, normalizes outputs, etc.
        BlobDetector blobs = new BlobDetector(purpleProc, greenProc, frameSize);

        // Telemetry and counters for captured pieces (by color).
        TelemetryHelper tele = new TelemetryHelper(this, true);
        int purpleCount = 0;
        int greenCount = 0;

        // State machine bookkeeping.
        TestState state = TestState.RUNNING;
        Timer dwellTimer = new Timer();

        // Runtime toggle debounce for moveChaseEnabled (X button)
        boolean lastX = false;

        // Early exit path if opmode is canceled before start; tidy up the vision portal.
        if (isStopRequested()) {
            try {
                portal.close();
            } catch (Exception ignored) {
            }
            return;
        }
        waitForStart();

        try {
            // Respect initial "chase/move" setting by enabling or disabling the drive follower PIDFs.
            if (!moveChaseEnabled) {
                drive.getFollower().deactivateAllPIDFs();
                drive.setAutoDrive(0, 0, 0, true, 0);
            } else {
                drive.getFollower().activateAllPIDFs();
            }

            while (opModeIsActive()) {
                // Runtime toggle for chase/move: press X to flip modes. Simple edge-detect debounce.
                boolean xNow = gamepad1.x;
                if (xNow && !lastX) {
                    moveChaseEnabled = !moveChaseEnabled;
                    if (!moveChaseEnabled) {
                        drive.getFollower().deactivateAllPIDFs();
                        drive.setAutoDrive(0, 0, 0, true, 0);
                    } else {
                        drive.getFollower().activateAllPIDFs();
                    }
                }
                lastX = xNow;

                // Update intake color sensor (blockingless; should be quick).
                intakeColor.update();

                // Intake mode ties to state machine: forward while RUNNING, off while dwelling.
                if (state == TestState.RUNNING) {
                    intake.setAutoMode(AutoMode.FORWARD);
                } else {
                    intake.setAutoMode(AutoMode.OFF);
                }
                intake.operate();

                // Command mirrors for telemetry only (so drivers can see what the loop requested).
                double commandedForward;
                double commandedStrafe;
                double commandedTurn;

                // Movement logic: pick the largest visible blob, steer toward its normalized center.
                if (state == TestState.RUNNING && moveChaseEnabled) {
                    // Query current best candidates
                    BlobDetector.Blob pb = blobs.bestPurpleBlob();
                    BlobDetector.Blob gb = blobs.bestGreenBlob();
                    BlobDetector.Blob chosen = pickLargest(pb, gb);

                    if (chosen.seen) {
                        // Convert normalized [-1..1] offsets into velocity commands with gain and clamps.
                        double vx = clamp(chosen.cxNorm * BLOB_APPROACH_GAIN, -BLOB_STRAFE_MAX, BLOB_STRAFE_MAX);
                        double vy = clamp(chosen.cyNorm * BLOB_APPROACH_GAIN, -BLOB_FORWARD_MAX, BLOB_FORWARD_MAX);

                        // Apply translation while holding a fixed field heading (faceHalf).
                        motion.translateFacing(vx, vy, faceHalf);

                        // For readability, record the requested drive components (sign note: forward is -vy here).
                        commandedForward = -vy;
                        commandedStrafe = -vx;
                        // Heading control happens internally; we don't expose commandedTurn here.
                    } else {
                        // No target: hold heading and zero translation.
                        motion.translateFacing(0, 0, faceHalf);
                        commandedForward = 0;
                        commandedStrafe = 0;
                    }
                    commandedTurn = Double.NaN; // indicates "PD inside controller" to telemetry
                } else {
                    // Movement disabled or dwelling: zero outputs, keep pose stable.
                    drive.setAutoDrive(0, 0, 0, true, 0);
                    commandedForward = 0;
                    commandedStrafe = 0;
                    commandedTurn = 0;
                }

                // Capture via color sensor: bump counts and enter dwell if either target color is seen.
                boolean purpleSeen = intakeColor.isPurple();
                boolean greenSeen = intakeColor.isGreen();

                if (state == TestState.RUNNING && (purpleSeen || greenSeen)) {
                    if (purpleSeen) purpleCount++;
                    if (greenSeen) greenCount++;
                    state = TestState.CAPTURED_DWELL;
                    dwellTimer.resetTimer();
                }

                // Dwell state: full stop for a fixed duration, then resume RUNNING.
                if (state == TestState.CAPTURED_DWELL) {
                    drive.setAutoDrive(0, 0, 0, true, 0);
                    if (dwellTimer.getElapsedTimeSeconds() > CAPTURE_DWELL_S) {
                        state = TestState.RUNNING;
                    }
                }

                // Push one iteration of the drivetrain operation.
                drive.operate();

                // Telemetry block: structured readout of mode, blobs, commands, and color captures.
                tele.addLine("=== INTAKE TEST ===")
                        .addData("Alliance", isRed ? "RED" : "BLUE")
                        .addData("Move/Chase Enabled", "%b", moveChaseEnabled)
                        .addData("State", state::name)
                        .addData("Portal FPS", "%.1f", portal.getFps());

                BlobDetector.Blob pb = blobs.bestPurpleBlob();
                BlobDetector.Blob gb = blobs.bestGreenBlob();
                tele.addLine("--- Blob (Purple) ---")
                        .addData("Seen", "%b", pb.seen)
                        .addData("cx,cy (norm)", "(%.2f, %.2f)", pb.cxNorm, pb.cyNorm)
                        .addData("Radius (norm)", "%.3f", pb.radiusNorm)
                        .addData("Area (px^2)", "%d", pb.area);

                tele.addLine("--- Blob (Green) ---")
                        .addData("Seen", "%b", gb.seen)
                        .addData("cx,cy (norm)", "(%.2f, %.2f)", gb.cxNorm, gb.cyNorm)
                        .addData("Radius (norm)", "%.3f", gb.radiusNorm)
                        .addData("Area (px^2)", "%d", gb.area);

                tele.addLine("--- Commands ---")
                        .addData("Drive.forward", "%.2f", commandedForward)
                        .addData("Drive.strafe", "%.2f", commandedStrafe)
                        .addData("Drive.turn", (Double.isNaN(commandedTurn) ? "PD" : String.format("%.2f", commandedTurn)));

                tele.addLine("--- Intake Color ---")
                        .addData("Is Purple?", "%b", purpleSeen)
                        .addData("Is Green?", "%b", greenSeen)
                        .addData("Captured", "P:%d G:%d", purpleCount, greenCount);

                TelemetryHelper.update();
                sleep(20); // ~50 Hz update rate; adjust for your robot if needed.
            }
        } finally {
            // Always try to close the vision portal to release the camera cleanly.
            try {
                portal.close();
            } catch (Exception ignored) {
            }
        }
    }

    /**
     * Pick the blob with the largest area among those actually seen.
     * If none are seen, returns the canonical "none" blob.
     */
    private static BlobDetector.Blob pickLargest(BlobDetector.Blob a, BlobDetector.Blob b) {
        if (a.seen && b.seen) return (a.area >= b.area) ? a : b;
        if (a.seen) return a;
        if (b.seen) return b;
        return BlobDetector.Blob.none();
    }

    /**
     * Clamp helper with clear bounds; tiny, testable, and prevents runaway commands.
     */
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
