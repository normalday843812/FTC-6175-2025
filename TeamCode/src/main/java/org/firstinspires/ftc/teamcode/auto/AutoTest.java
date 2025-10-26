package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_BLUE_AUDIENCE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_BLUE_DEPOT;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_RED_AUDIENCE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_RED_DEPOT;

import android.graphics.Color;
import android.util.Size;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.deposit.DepositController;
import org.firstinspires.ftc.teamcode.auto.intake.IntakeController;
import org.firstinspires.ftc.teamcode.auto.inventory.AutoInventory;
import org.firstinspires.ftc.teamcode.auto.motion.AllianceGoalHeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.FixedFieldHeading;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingController;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.auto.phases.MovePhaseController;
import org.firstinspires.ftc.teamcode.auto.phases.MovePhaseController.Result;
import org.firstinspires.ftc.teamcode.auto.search.SearchController;
import org.firstinspires.ftc.teamcode.auto.state.AutoState;
import org.firstinspires.ftc.teamcode.auto.vision.BlobDetector;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.Menu;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

@Autonomous(name = "Auto test", group = "Pedro")
public class AutoTest extends LinearOpMode {
    public static boolean isRed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Menu menu = new Menu(this)
                .add(new Menu.Item(
                        "Alliance",
                        "Red (B)", () -> gamepad1.b,
                        "Blue (X)", () -> gamepad1.x,
                        true))
                .add(new Menu.Item(
                        "Side",
                        "Audience", () -> gamepad1.y,
                        "Depot", () -> gamepad1.a,
                        true
                ))
                .add(new Menu.Item(
                        "Actively intake?",
                        "Yes (A)", () -> gamepad1.a,
                        "No (B)", () -> gamepad1.b,
                        true))
                .add(new Menu.Item(
                        "Shoot Preloaded?",
                        "Yes (A)", () -> gamepad1.a,
                        "No (B)", () -> gamepad1.b,
                        true
                ))
                .add(new Menu.Item(
                        "Move?",
                        "Yes (A)", () -> gamepad1.a,
                        "No (B)", () -> gamepad1.b,
                        true
                ));

        menu.showUntilStart();

        isRed = menu.get("Alliance");
        boolean isAudienceSide = menu.get("Side");
        boolean activelyIntake = menu.get("Actively intake?");
        boolean shootPreloaded = menu.get("Shoot Preloaded?");
        boolean move = menu.get("Move?");

        Pose startPose = pickStartPose(isRed, isAudienceSide);

        // Hardware and subsystems
        RobotHardware hw = new RobotHardware(this);
        hw.initLimeLight(100);
        hw.initWebcam();
        hw.initIntake();
        hw.initShooter();
        hw.initSpindexer();
        hw.initIntakeColorSensor();
        hw.initTransfer();

        Mecanum drive = new Mecanum(this, null);
        drive.init();
        drive.setStartingPose(startPose);
        drive.startAuto();

        Shooter shooter = new Shooter(hw.getShooterMotor(), null, this);
        shooter.startAuto();

        Intake intake = new Intake(hw.getIntakeMotor(), null, this);
        intake.startAuto();

        Spindexer spindexer = new Spindexer(hw.getSpindexerServo(), null, this);
        spindexer.startAuto();

        Transfer transfer = new Transfer(hw.getTransferServo(), null, this);
        transfer.startAuto();

        IntakeColorSensor intakeColor = new IntakeColorSensor(hw.getIntakeColorSensor(), this);

        // ===== Motion/heading =====
        HeadingController headingCtrl = new HeadingController();
        TelemetryHelper motionTele = new TelemetryHelper(this, true);
        MotionController motion = new MotionController(drive, headingCtrl, motionTele);

        Limelight3A limelight = hw.getLimelight();
        HeadingTarget goalTarget = new AllianceGoalHeadingTarget(limelight, isRed);
        HeadingTarget fallbackTarget = new FixedFieldHeading(isRed ? 0 : 180, "Fallback");

        // ===== Move =====
        TelemetryHelper moveTele = new TelemetryHelper(this, true);
        MovePhaseController movePhase = new MovePhaseController(
                motion,
                fallbackTarget,
                isRed,
                isAudienceSide,
                move,
                shootPreloaded,
                activelyIntake,
                moveTele
        );

        // Deposit
        TelemetryHelper depositTele = new TelemetryHelper(this, true);
        DepositController deposit = new DepositController(
                shooter, spindexer, transfer, motion, goalTarget, isRed, depositTele
        );

        // Vision stuff
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

        Size frameSize = new Size(320, 240);
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(purpleProc)
                .addProcessor(greenProc)
                .setCameraResolution(frameSize)
                .setCamera(hw.getWebcam1())
                .build();

        BlobDetector blobDetector = new BlobDetector(purpleProc, greenProc, frameSize);

        // Inventory and controllers
        AutoInventory inventory = new AutoInventory(2);

        TelemetryHelper searchTele = new TelemetryHelper(this, true);
        SearchController search = new SearchController(motion, blobDetector, inventory, isRed, searchTele);

        TelemetryHelper intakeTele = new TelemetryHelper(this, true);
        IntakeController intakeCtrl = new IntakeController(motion, blobDetector, inventory, intake, intakeColor, isRed, intakeTele);

        AutoState state = AutoState.MOVE;

        if (isStopRequested()) {
            try {
                portal.close();
            } catch (Exception ignored) {
            }
            return;
        }
        waitForStart();

        try {
            while (opModeIsActive()) {
                intakeColor.update();
                intake.operate();
                transfer.operate();

                // State step
                switch (state) {
                    case MOVE: {
                        Pose current = drive.getFollower().getPose();
                        Result res = movePhase.update(current);
                        if (res == Result.TO_DEPOSIT) {
                            state = AutoState.DEPOSIT;
                            deposit.startCycle();
                        } else if (res == Result.TO_DONE) {
                            state = activelyIntake ? AutoState.SEARCH : AutoState.DONE;
                            deposit.startCycle();
                        }
                        break;
                    }
                    case DEPOSIT: {
                        boolean done = deposit.update();
                        if (done) {
                            inventory.clear();
                            state = activelyIntake ? AutoState.SEARCH : AutoState.DONE;
                        }
                        break;
                    }
                    case SEARCH: {
                        Pose current = drive.getFollower().getPose();
                        SearchController.Result r = search.update(current);
                        if (r == SearchController.Result.TO_DEPOSIT) {
                            state = AutoState.DEPOSIT;
                        } else if (r == SearchController.Result.TO_INTAKE) {
                            intakeCtrl.setTargetColor(search.lastChosenIsPurple());
                            state = AutoState.INTAKE;
                        }
                        break;
                    }
                    case INTAKE: {
                        Pose current = drive.getFollower().getPose();
                        IntakeController.Result r = intakeCtrl.update(current);
                        if (r == IntakeController.Result.TO_SEARCH) {
                            state = AutoState.SEARCH;
                            search.resetBudget();
                        } else if (r == IntakeController.Result.TO_DEPOSIT) {
                            state = AutoState.DEPOSIT;
                        }
                        break;
                    }
                    case DONE: {
                        // Stop drive
                        drive.setAutoDrive(0, 0, 0, true, 0);
                        // Stop shooter
                        shooter.setAutoRpm(0);
                        shooter.operate();
                        intake.setAutoMode(Intake.AutoMode.OFF);
                        break;
                    }
                }

                // Drive step
                drive.operate();

                // Telemetry
                TelemetryHelper.update();
                sleep(20);
            }
        } finally {
            try {
                portal.close();
            } catch (Exception ignored) {
            }
        }
    }

    private Pose pickStartPose(boolean isRed, boolean isAudienceSide) {
        if (isRed) return isAudienceSide ? START_RED_AUDIENCE : START_RED_DEPOT;
        return isAudienceSide ? START_BLUE_AUDIENCE : START_BLUE_DEPOT;
    }
}
