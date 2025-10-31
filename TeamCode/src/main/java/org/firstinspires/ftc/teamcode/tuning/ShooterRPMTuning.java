package org.firstinspires.ftc.teamcode.tuning;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.EdgeTrigger;
import org.firstinspires.ftc.teamcode.util.Menu;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

/**
 * Shooter RPM Tuning OpMode
 *
 * What it does:
 * - Drive the robot with your existing Mecanum + GamepadMap controller (normal teleop)
 * - Control shooter RPM manually with triggers (uses your Shooter subsystem as-is)
 * - Press dpad_up to CAPTURE a pending sample at that instant:
 *      (pose, motor RPM) are recorded and transfer is flicked
 *      If a previous sample is pending without a label, it's discarded.
 * - Press dpad_right to label the pending sample as HIT.
 * - Press dpad_left  to label the pending sample as MISS.
 *      Optionally include misses in regression with a positive margin (see constants).
 * - After each labeled sample, a 5-term polynomial model is refit by least-squares:
 *      RPM ≈ a0 + a1*D + a2*D^2 + b1*theta + b2*theta^2
 *      (D in inches; theta is abs heading error in radians)
 * - Telemetry shows sample counts and current coefficients (pasteable).
 *
 */
@TeleOp(name = "ShooterRPM_Tuning", group = "Tuning")
public class ShooterRPMTuning extends LinearOpMode {

    // MADE BY CHATGPT MAY NOT WORK

    // ------------------------------
    // Tuning constants (inline)
    // ------------------------------
    // Goal location (Pedro coords). Pick rough center of alliance goal on the back wall.
    private static final double GOAL_RED_X  = 140.0;
    private static final double GOAL_RED_Y  = 72.0;
    private static final double GOAL_BLUE_X = 4.0;
    private static final double GOAL_BLUE_Y = 72.0;

    // Include heading error terms in the model
    private static final boolean INCLUDE_HEADING_TERMS = true;

    // Use only HITs (recommended) OR turn this on to include MISS with a positive margin:
    private static final boolean INCLUDE_MISS_AS_CONSTRAINT = true;
    private static final double  MISS_RPM_MARGIN            = 150.0;  // rpm to add to a MISS sample

    // Minimal number of samples required to solve the 5-coefficient model
    private static final int MIN_SAMPLES_TO_SOLVE = 5;

    // Print model-predicted RPM for current pose (if we have a solution)
    private static final boolean SHOW_PREDICTED_FOR_CURRENT_POSE = true;

    // ------------------------------
    // Model dimension: a0, a1*D, a2*D^2, b1*theta, b2*theta^2
    // ------------------------------
    private static final int FEATURE_DIM = 5;

    // Accumulated normal equations: S = sum(x x^T), b = sum(x * y)
    private double[][] S = new double[FEATURE_DIM][FEATURE_DIM];
    private double[]   B = new double[FEATURE_DIM];

    private long totalSamples = 0; // labeled samples used in S,B
    private long hitCount = 0;
    private long missCount = 0;

    // Latest solution vector w (length 5). Null if not solved yet.
    private double[] coeffs = null;

    // ------------------------------
    // Subsystems
    // ------------------------------
    private RobotHardware hw;
    private GamepadMap map;
    private Mecanum drive;
    private Shooter shooter;
    private Intake intake;
    private Transfer transfer;
    private TelemetryHelper tele;

    // ------------------------------
    // Goal selection
    // ------------------------------
    private boolean isRed = true; // chosen by menu at init time
    private Pose goalPose;

    // ------------------------------
    // Pending sample mechanism
    // ------------------------------
    private static class Pending {
        Pose pose;
        double dist;
        double theta;
        double motorRpm;
        long timestampMs;

        Pending(Pose p, double d, double t, double rpm, long ts) {
            pose = p; dist = d; theta = t; motorRpm = rpm; timestampMs = ts;
        }
    }
    private Pending pending = null;

    // Edge triggers for local controls (we are NOT letting Transfer map flick by itself)
    private final EdgeTrigger captureEdge = new EdgeTrigger();   // dpad_up -> capture & flick
    private final EdgeTrigger labelHitEdge = new EdgeTrigger();  // dpad_right -> label HIT
    private final EdgeTrigger labelMissEdge = new EdgeTrigger(); // dpad_left -> label MISS
    private final EdgeTrigger clearEdge = new EdgeTrigger();     // back+start -> clear dataset

    @Override
    public void runOpMode() throws InterruptedException {
        // ------------------------------------
        // Simple menu to pick alliance for goal
        // ------------------------------------
        Menu menu = new Menu(this)
                .add(new Menu.Item(
                        "Alliance",
                        "Red (B)",  () -> gamepad1.b,
                        "Blue (X)", () -> gamepad1.x,
                        true));
        menu.showUntilStart();
        isRed = menu.get("Alliance");
        if (isRed) {
            goalPose = new Pose(GOAL_RED_X, GOAL_RED_Y, 0);
        } else {
            goalPose = new Pose(GOAL_BLUE_X, GOAL_BLUE_Y, 0);
        }

        // ------------------------------------
        // Hardware and subsystems
        // ------------------------------------
        hw  = new RobotHardware(this);
        map = new GamepadMap(this);

        tele = new TelemetryHelper(this, true);

        // Drivetrain (Pedro follower inside). This uses your Constants.createFollower() pipeline.
        drive = new Mecanum(this, map);
        drive.init();
        drive.startTeleop();

        // Shooter (manual mode via triggers in Shooter.operate())
        hw.initShooter();
        shooter = new Shooter(hw.getShooterMotor(), map, this);
        shooter.startTeleop(); // manual

        // Intake (not crucial but we enable it so you can run as normal)
        hw.initIntake();
        intake = new Intake(hw.getIntakeMotor(), map, this);
        intake.startTeleop();

        // Transfer: we will control flick explicitly from this OpMode, so pass null for the map
        hw.initTransfer();
        transfer = new Transfer(hw.getTransferServo1(), hw.getTransferServo2(), null, this);
        transfer.startTeleop();

        if (isStopRequested()) return;
        waitForStart();

        resetDataset();

        while (opModeIsActive()) {
            // Update driver controls, but transfer flick is managed here
            map.update();

            // Normal teleop operations
            drive.operate();     // drive subsystem (handles field-centric etc.)
            intake.operate();
            shooter.operate();   // manual triggers set shooter RPM
            transfer.operate();  // runs flick state machine if we command flick()

            // Handle capture/flick and labeling with our own edges
            handleCaptureAndLabel();

            // Attempt to solve after any sample updates (non-blocking)
            solveIfPossible();

            // Telemetry
            addTelemetry();
            TelemetryHelper.update();

            sleep(20);
        }
    }

    // ------------------------------------
    // Sample capture and labeling
    // ------------------------------------
    private void handleCaptureAndLabel() {
        // Clear dataset with back+start if desired
        boolean clear = gamepad1.back && gamepad1.start;
        if (clearEdge.rose(clear)) {
            resetDataset();
        }

        // 1) Capture pending sample and flick (on dpad_up)
        if (captureEdge.rose(gamepad1.right_stick_button)) {
            // Discard previous pending (if any)
            pending = null;

            // Build pending snapshot: pose, distance to goal, abs heading error, motor RPM
            Pose cur = drive.getFollower().getPose();
            double dx = goalPose.getX() - cur.getX();
            double dy = goalPose.getY() - cur.getY();
            double dist = Math.hypot(dx, dy);

            double desiredHeading = Math.atan2(dy, dx);
            double headingErr = Math.abs(
                    MathFunctions.getSmallestAngleDifference(cur.getHeading(), desiredHeading)
            );

            double motorRpm = shooter.getMotorRPM(); // capture motor RPM at this instant

            pending = new Pending(cur, dist, headingErr, motorRpm, System.currentTimeMillis());

            // IMPORTANT: Flick now to keep the workflow consistent (capture occurs exactly when flick is pressed)
            transfer.flick();
        }

        // 2) Label the pending sample as HIT or MISS
        if (pending != null) {
            if (labelHitEdge.rose(gamepad1.dpad_right)) {
                addLabeledSample(pending, true);
                pending = null;
            } else if (labelMissEdge.rose(gamepad1.dpad_left)) {
                addLabeledSample(pending, false);
                pending = null;
            }

            // If user hits capture again before labeling, the new capture handler above discards previous pending automatically.
        }
    }

    private void resetDataset() {
        for (int i = 0; i < FEATURE_DIM; i++) {
            for (int j = 0; j < FEATURE_DIM; j++) {
                S[i][j] = 0.0;
            }
            B[i] = 0.0;
        }
        totalSamples = 0;
        hitCount = 0;
        missCount = 0;
        coeffs = null;
        pending = null;
    }

    // Adds a labeled sample to the normal equations (X^T X and X^T y)
    private void addLabeledSample(Pending p, boolean isHit) {
        // Construct feature vector x = [1, D, D^2, theta, theta^2]
        double[] x = makeFeatureVector(p.dist, p.theta);

        // Decide y for this sample:
        // - If HIT: use the actual motor RPM at capture
        // - If MISS and INCLUDE_MISS_AS_CONSTRAINT: use (motor RPM + margin) to push model upward
        // - Otherwise (MISS & not included): do nothing
        if (!isHit && !INCLUDE_MISS_AS_CONSTRAINT) {
            missCount++;
            return; // skip MISS if we're not including them
        }

        double y = isHit ? p.motorRpm : (p.motorRpm + MISS_RPM_MARGIN);

        // Update S += x x^T, B += x y
        for (int i = 0; i < FEATURE_DIM; i++) {
            B[i] += x[i] * y;
            for (int j = 0; j < FEATURE_DIM; j++) {
                S[i][j] += x[i] * x[j];
            }
        }

        totalSamples++;
        if (isHit) hitCount++; else missCount++;
    }

    private double[] makeFeatureVector(double dist, double thetaErr) {
        // Model: [1, D, D^2, theta, theta^2]
        double[] x = new double[FEATURE_DIM];
        x[0] = 1.0;
        x[1] = dist;
        x[2] = dist * dist;

        if (INCLUDE_HEADING_TERMS) {
            x[3] = thetaErr;
            x[4] = thetaErr * thetaErr;
        } else {
            x[3] = 0.0;
            x[4] = 0.0;
        }
        return x;
    }

    // Try to solve (X^T X) w = (X^T y) by RREF
    private void solveIfPossible() {
        // Need at least min samples to solve a 5-term model
        if (totalSamples < MIN_SAMPLES_TO_SOLVE) return;

        try {
            Matrix A = new Matrix(S);              // 5x5
            Matrix b = new Matrix(FEATURE_DIM, 1); // 5x1
            for (int i = 0; i < FEATURE_DIM; i++) b.set(i, 0, B[i]);

            // Solve by RREF. Returns [reducedA, reducedB]
            Matrix[] r = Matrix.rref(A, b);
            Matrix reducedA = r[0];
            Matrix reducedB = r[1];

            // If reducedA is near identity, reducedB is the solution
            double[] w = new double[FEATURE_DIM];
            for (int i = 0; i < FEATURE_DIM; i++) {
                w[i] = reducedB.get(i, 0);
            }
            coeffs = w;

        } catch (Exception e) {
            // Could be singular or ill-conditioned; keep old coeffs
            // Telemetry shows "coeffs null" if not solved
        }
    }

    private double predictRpmForPose(Pose current) {
        if (coeffs == null) return 0.0;
        double dx = goalPose.getX() - current.getX();
        double dy = goalPose.getY() - current.getY();
        double dist = Math.hypot(dx, dy);
        double desiredHeading = Math.atan2(dy, dx);
        double theta = Math.abs(MathFunctions.getSmallestAngleDifference(current.getHeading(), desiredHeading));
        double[] x = makeFeatureVector(dist, theta);

        double y = 0.0;
        for (int i = 0; i < FEATURE_DIM; i++) {
            y += coeffs[i] * x[i];
        }
        return y;
    }

    private void addTelemetry() {
        Pose p = drive.getFollower().getPose();
        double dx = goalPose.getX() - p.getX();
        double dy = goalPose.getY() - p.getY();
        double dist = Math.hypot(dx, dy);
        double desiredHeading = Math.atan2(dy, dx);
        double theta = Math.abs(MathFunctions.getSmallestAngleDifference(p.getHeading(), desiredHeading));

        tele.addLine("=== Shooter RPM Tuning ===")
                .addData("Alliance", isRed ? "RED" : "BLUE")
                .addData("Goal", "(%.1f, %.1f)", goalPose.getX(), goalPose.getY())
                .addData("Pose", "(x=%.1f, y=%.1f, h=%.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()))
                .addData("Dist to Goal", "%.2f in", dist)
                .addData("Heading Err", "%.3f rad", theta)
                .addData("Motor RPM", "%.0f", shooter.getMotorRPM());

        // Pending sample display
        if (pending != null) {
            tele.addLine("--- Pending ---")
                    .addData("RPM", "%.0f", pending.motorRpm)
                    .addData("Dist", "%.2f", pending.dist)
                    .addData("Theta", "%.3f", pending.theta)
                    .addData("Age(ms)", "%d", System.currentTimeMillis() - pending.timestampMs);
        } else {
            tele.addLine("--- Pending ---").addData("Status", "none");
        }

        // Dataset stats
        tele.addLine("--- Dataset ---")
                .addData("Total", "%d", totalSamples)
                .addData("Hits",  "%d", hitCount)
                .addData("Miss",  "%d", missCount)
                .addData("MinSamplesToSolve", "%d", MIN_SAMPLES_TO_SOLVE);

        // Coeffs (pasteable)
        if (coeffs != null) {
            double a0 = coeffs[0], a1 = coeffs[1], a2 = coeffs[2], b1 = coeffs[3], b2 = coeffs[4];
            tele.addLine("--- Model (RPM ≈ a0 + a1*D + a2*D^2 + b1*theta + b2*theta^2) ---")
                    .addData("a0", "%.3f", a0)
                    .addData("a1", "%.3f", a1)
                    .addData("a2", "%.3f", a2)
                    .addData("b1", "%.3f", b1)
                    .addData("b2", "%.3f", b2);

            if (SHOW_PREDICTED_FOR_CURRENT_POSE) {
                tele.addData("Predicted @pose", "%.0f rpm", predictRpmForPose(p));
            }

            tele.addData("A0", "%.6f", a0)
                    .addData("A1", "%.6f", a1)
                    .addData("A2", "%.6f", a2)
                    .addData("B1", "%.6f", b1)
                    .addData("B2", "%.6f", b2);
        } else {
            tele.addLine("--- Model ---").addData("Coeffs", "null (not solved yet)");
        }

        // Controls cheat-sheet
        tele.addLine("--- Controls ---")
                .addLine("Drive: normal Mecanum teleop")
                .addLine("Shooter: triggers (manual as in Shooter.operate())")
                .addLine("Capture+Flick: dpad_up")
                .addLine("Label:   HIT dpad_right | MISS dpad_left")
                .addLine("Clear dataset: back+start");
    }
}