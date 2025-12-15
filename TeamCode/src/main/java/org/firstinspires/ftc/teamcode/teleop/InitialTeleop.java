package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.isRed;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.config.LLAprilTagConfig;
import org.firstinspires.ftc.teamcode.config.ShooterYawConfig;
import org.firstinspires.ftc.teamcode.managers.InventoryManager;
import org.firstinspires.ftc.teamcode.managers.PersistentBallState;
import org.firstinspires.ftc.teamcode.managers.PersistentPoseState;
import org.firstinspires.ftc.teamcode.managers.TeleopManager;
import org.firstinspires.ftc.teamcode.managers.UiLight;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.RgbIndicator;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterYaw;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.vision.LLAprilTag;

@TeleOp
public class InitialTeleop extends LinearOpMode {
    boolean managerEnabled = true;

    private static final double METERS_TO_INCHES = 39.3700787402;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(true);

        RobotHardware hw = new RobotHardware(this);

        GamepadMap map = new GamepadMap(this);

        // Drive
        Mecanum drive = new Mecanum(this, map);
        drive.init();
        if (PersistentPoseState.isInitialized()) {
            drive.setStartingPose(PersistentPoseState.loadPose());
        }

        // Subsystems
        hw.initIntake();
        Intake intake = new Intake(hw.getIntakeMotor(), this);

        hw.initShooter();
        Shooter shooter = new Shooter(hw.getShooterMotor(), hw.getShooterMotor1(), this);

        hw.initShooterYaw();
        ShooterYaw shooterYaw = new ShooterYaw(hw.getShooterYawMotor(), drive.getFollower(), isRed, this);
        shooterYaw.setGoalTrackingEnabled(ShooterYawConfig.TELEOP_GOAL_TRACKING_ENABLED);
        shooterYaw.setAimPoseOverrideEnabled(ShooterYawConfig.TELEOP_USE_LL_POSE_FOR_AIMING);
        shooterYaw.setAimPoseOverrideTtlMs(ShooterYawConfig.TELEOP_LL_POSE_TTL_MS);

        hw.initSpindexer();
        Spindexer spindexer = new Spindexer(hw.getSpindexerServo(), this);

        hw.initSpindexColorSensors();
        SlotColorSensors slots = new SlotColorSensors(hw.getSpindexSensors(), this);

        hw.initTransfer();
        Transfer transfer = new Transfer(hw.getTransferServo1(),
                hw.getTransferServo2(), this);

        hw.initRgbIndicator();
        RgbIndicator rgbIndicator = new RgbIndicator(hw.getRgbIndicator());
        UiLight ui = new UiLight(rgbIndicator);

        InventoryManager inventoryManager = new InventoryManager();

        // Load persistent ball state from auto (if available)
        if (PersistentBallState.isInitialized()) {
            PersistentBallState.loadIntoModel(inventoryManager.getModel());
            telemetry.addLine("Loaded ball state from auto");
            telemetry.addData("Balls", PersistentBallState.getBallCount());
        } else {
            telemetry.addLine("No ball state from auto - starting fresh");
        }
        if (PersistentPoseState.isInitialized()) {
            telemetry.addLine("Loaded pose from auto");
            telemetry.addData("Pose", PersistentPoseState.getPoseString());
        } else {
            telemetry.addLine("No pose from auto - starting at origin");
        }
        telemetry.update();

        TeleopManager teleopManager =
                new TeleopManager(intake, shooter, shooterYaw, spindexer, transfer, slots, inventoryManager, ui, this);

        // --- Optional: Limelight AprilTag relocalization (before Start) ---
        LLAprilTag llAprilTag = null;
        boolean llPoseApplied = false;
        if (LLAprilTagConfig.ENABLED) {
            try {
                hw.initLimeLight(LLAprilTagConfig.POLL_RATE_HZ);
                hw.setLimelightPipeline(LLAprilTagConfig.PIPELINE);
                llAprilTag = new LLAprilTag(hw.getLimelight(), this);
                teleopManager.setLlAprilTag(llAprilTag, isRed);
            } catch (Throwable t) {
                telemetry.addLine("Limelight init failed; skipping relocalization");
                telemetry.addData("LL Error", "%s", t.getClass().getSimpleName());
                telemetry.update();
            }
        }

        // Start ShooterYaw early so we can hold/scan it during init if desired.
        shooterYaw.start();

        double scanTicks = org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MIN_TICKS;
        int scanDir = 1;
        Timer scanDt = new Timer();

        while (!isStarted() && !isStopRequested()) {
            if (llAprilTag != null) {
                llAprilTag.update();
                if (!llPoseApplied) {
                    LLAprilTag.YawInfo yaw = llAprilTag.getYawInfoForAllianceHome(isRed);
                    Pose3D botpose = llAprilTag.getBotPose();
                    if (yaw.fresh && botpose != null) {
                        Pose pedroPose = toPedroPoseFromLLBotpose(botpose);
                        // Teleop uses Limelight MegaTag pose to help ShooterYaw aim, but does NOT relocalize odometry.
                        shooterYaw.setAimPoseOverride(pedroPose);
                        llPoseApplied = true;
                    }
                }
            }

            // Scan ShooterYaw to acquire the goal tag (Limelight is mounted on ShooterYaw).
            if (llAprilTag != null
                    && !llPoseApplied
                    && org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TELEOP_GOAL_TRACKING_ENABLED
                    && org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TELEOP_TAG_SCAN_ENABLED) {
                double dt = scanDt.getElapsedTimeSeconds();
                scanDt.resetTimer();
                dt = Math.max(0.0, Math.min(0.05, dt));

                scanTicks += scanDir * org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TELEOP_TAG_SCAN_TICKS_PER_S * dt;
                if (scanTicks >= org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MAX_TICKS) {
                    scanTicks = org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MAX_TICKS;
                    scanDir = -1;
                } else if (scanTicks <= org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MIN_TICKS) {
                    scanTicks = org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MIN_TICKS;
                    scanDir = 1;
                }
                shooterYaw.holdTicks((int) Math.round(scanTicks));
            } else {
                shooterYaw.holdCenter();
            }

            shooterYaw.operate();

            telemetry.addLine("=== TELEOP INIT ===");
            telemetry.addData("PoseFromAuto", "%b", PersistentPoseState.isInitialized());
            if (PersistentPoseState.isInitialized()) telemetry.addData("AutoPose", PersistentPoseState.getPoseString());
            telemetry.addData("LL Enabled", "%b", llAprilTag != null);
            telemetry.addData("LL Pose Applied", "%b", llPoseApplied);
            if (llAprilTag != null) {
                LLAprilTag.YawInfo yaw = llAprilTag.getYawInfoForAllianceHome(isRed);
                telemetry.addData("TagFresh", "%b", yaw.fresh);
                telemetry.addData("TagYaw(avg)", "%.1f", yaw.avgDeg);
                telemetry.addData("TagAgeMs", "%d", yaw.ageMs);
            }
            telemetry.update();
            idle();
        }

        if (isStopRequested()) return;
        waitForStart();

        drive.startTeleop();
        shooter.start();

        spindexer.start();
        // Restore the spindexer slot from the persisted model so hardware + model stay aligned.
        // This prevents the manager from thinking "bucketAtFront = X" while the servo is actually at slot 0.
        try {
            int desiredSlot = inventoryManager.getModel().getBucketAtFront();
            spindexer.setSlot(desiredSlot);

            long startMs = System.currentTimeMillis();
            while (opModeIsActive()
                    && !spindexer.isSettled()
                    && (System.currentTimeMillis() - startMs) < 400) {
                spindexer.operate();
                idle();
            }
        } catch (Throwable t) {
            // Ignore; fall back to slot 0 if anything goes wrong.
        }
        transfer.start();
        shooterYaw.start(); // switch back to normal tracking mode in teleop

        teleopManager.setEnabled(managerEnabled);

        while (opModeIsActive()) {
            map.update();

            boolean prevManagerEnabled = managerEnabled;
            if (map.teleopSortManagerToggle) {
                managerEnabled = !managerEnabled;
            }

            if (managerEnabled != prevManagerEnabled) {
                teleopManager.setEnabled(managerEnabled);
            }

            // Drive first so shooter yaw sees the latest pose.
            drive.operate();
            teleopManager.update(map);

            // Save ball state periodically so it persists
            PersistentBallState.saveFromModel(inventoryManager.getModel());
            PersistentPoseState.saveFromPose(drive.getFollower().getPose());

            TelemetryHelper.update();
        }
    }

    private static Pose toPedroPoseFromLLBotpose(Pose3D botpose) {
        double xIn = botpose.getPosition().x * METERS_TO_INCHES;
        double yIn = botpose.getPosition().y * METERS_TO_INCHES;
        double headingRad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        // Limelight botpose is in FTC standard coordinates; convert to Pedro coordinates.
        Pose ftcPose = new Pose(xIn, yIn, headingRad);
        return FTCCoordinates.INSTANCE.convertToPedro(ftcPose);
    }
}
