package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_BLUE_AUDIENCE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_BLUE_DEPOT;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_RED_AUDIENCE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_RED_DEPOT;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.deposit.DepositController;
import org.firstinspires.ftc.teamcode.auto.motion.AllianceGoalHeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.FixedFieldHeading;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingController;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.auto.phases.MovePhaseController;
import org.firstinspires.ftc.teamcode.auto.phases.MovePhaseController.Result;
import org.firstinspires.ftc.teamcode.auto.state.AutoState;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Menu;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@Autonomous(name = "Auto test", group = "Pedro")
public class AutoTest extends LinearOpMode {
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

        boolean isRed = menu.get("Alliance");
        boolean isAudienceSide = menu.get("Side");
        boolean activelyIntake = menu.get("Actively intake?");
        boolean shootPreloaded = menu.get("Shoot Preloaded?");
        boolean move = menu.get("Move?");

        Pose startPose = pickStartPose(isRed, isAudienceSide);

        // Hardware and subsystems
        RobotHardware hw = new RobotHardware(this);
        hw.initLimeLight(100);
        hw.initIntake();
        hw.initShooter();
        hw.initHood();
        hw.initSpindexer();

        Mecanum drive = new Mecanum(this, null);
        drive.init();
        drive.setStartingPose(startPose);
        drive.startAuto();

        Shooter shooter = new Shooter(hw.getShooterMotor(), null, this);
        shooter.startAuto();

        Intake intake = new Intake(hw.getIntakeMotor(), null, this);
        intake.startAuto();

        Hood hood = new Hood(hw.getHoodServo(), null, this);
        hood.startAuto();

        Spindexer spindexer = new Spindexer(hw.getSpindexerServo(), null, this);
        spindexer.startAuto();

        // Motion/heading
        HeadingController headingCtrl = new HeadingController();
        TelemetryHelper motionTele = new TelemetryHelper(this, true);
        MotionController motion = new MotionController(drive, headingCtrl, motionTele);

        Limelight3A limelight = hw.getLimelight();
        HeadingTarget goalTarget = new AllianceGoalHeadingTarget(limelight, isRed);
        HeadingTarget fallbackTarget = new FixedFieldHeading(isRed ? 0 : 180, "Fallback");

        TelemetryHelper moveTele = new TelemetryHelper(this, true);
        MovePhaseController movePhase = new MovePhaseController(
                motion,
                fallbackTarget,
                isRed,
                isAudienceSide,
                move,
                shootPreloaded,
                moveTele
        );

        TelemetryHelper depositTele = new TelemetryHelper(this, true);
        DepositController deposit = new DepositController(
                shooter, hood, spindexer, motion, goalTarget, isRed, depositTele
        );

        AutoState state = AutoState.MOVE;

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {
            intake.operate();

            // State step
            switch (state) {
                case MOVE: {
                    Pose current = drive.getFollower().getPose();
                    Result res = movePhase.update(current);
                    if (res == Result.TO_DEPOSIT) state = AutoState.DEPOSIT;
                    else if (res == Result.TO_DONE) state = AutoState.DONE;
                    break;
                }
                case DEPOSIT: {
                    boolean done = deposit.update();
                    if (done) state = AutoState.DONE;
                    break;
                }
                case DONE: {
                    // Stop drive
                    drive.setAutoDrive(0, 0, 0, true, 0);
                    // Stop shooter
                    shooter.setAutoRpm(0);
                    shooter.operate();
                    break;
                }
            }

            // Drive step
            drive.operate();

            // Telemetry
            TelemetryHelper.update();
            sleep(20);
        }
    }

    private Pose pickStartPose(boolean isRed, boolean isAudienceSide) {
        if (isRed) return isAudienceSide ? START_RED_AUDIENCE : START_RED_DEPOT;
        return isAudienceSide ? START_BLUE_AUDIENCE : START_BLUE_DEPOT;
    }
}