package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.isAudienceSide;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.isRed;
import static org.firstinspires.ftc.teamcode.config.SimpleAutoConfig.buildPlan;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.deposit.DepositController;
import org.firstinspires.ftc.teamcode.auto.motion.AllianceGoalHeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingController;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.config.SimpleAutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterYaw;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.Menu;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.vision.LLAprilTag;

@Autonomous(name = "Simple Auto", group = "Pedro")
public class SimpleAuto extends LinearOpMode {

    private enum State {
        DRIVE_TO_SHOOT,
        DEPOSIT,
        RETURN_HOME,
        DONE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Menu setup
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
                ));

        menu.showUntilStart();
        isRed = menu.get("Alliance");
        isAudienceSide = menu.get("Side");

        // Determine positions based on selection
        SimpleAutoConfig.PathPlan plan = buildPlan(isRed, isAudienceSide);
        Pose startPose = plan.startPose;

        // Hardware and subsystems
        RobotHardware hw = new RobotHardware(this);
        hw.initLimeLight(100);
        hw.initShooter();
        hw.initShooterYaw();
        hw.initSpindexer();
        hw.initTransfer();

        Mecanum drive = new Mecanum(this, null);
        drive.init();
        drive.setStartingPose(startPose);
        drive.startAuto();

        LLAprilTag ll = new LLAprilTag(hw.getLimelight(), this);
        HeadingTarget goalTarget = new AllianceGoalHeadingTarget(ll, isRed);

        Shooter shooter = new Shooter(hw.getShooterMotor(), null, this);
        shooter.startAuto();

        ShooterYaw shooterYaw = new ShooterYaw(hw.getShooterYawMotor(), ll, isRed,
                null, this);
        shooterYaw.startAuto();
        shooterYaw.setAutoLock(true);

        Spindexer spindexer = new Spindexer(hw.getSpindexerServo(), null, this);
        spindexer.startAuto();

        Intake intake = new Intake(hw.getIntakeMotor(), null, this);
        intake.startAuto();

        Transfer transfer = new Transfer(hw.getTransferServo1(), hw.getTransferServo2(),
                null, this);
        transfer.startAuto();

        // Motion setup
        HeadingController headingCtrl = new HeadingController();
        TelemetryHelper motionTele = new TelemetryHelper(this, true);
        MotionController motion = new MotionController(drive, headingCtrl, motionTele);

        // Deposit controller
        TelemetryHelper depositTele = new TelemetryHelper(this, true);
        DepositController deposit = new DepositController(
                shooter, spindexer, transfer, motion, goalTarget, isRed, depositTele
        );

        drive.getFollower().followPath(plan.toShootPath);
        drive.getFollower().followPath(plan.returnPath);

        if (isStopRequested()) return;

        waitForStart();
        deposit.update();

        sleep(10000);

        State state = State.DRIVE_TO_SHOOT;
        drive.getFollower().followPath(plan.toShootPath);

        while (opModeIsActive()) {
            ll.update();
            drive.operate();
            shooterYaw.operate();
            transfer.operate();
            intake.operate();
            intake.setAutoMode(Intake.AutoMode.FORWARD);

            switch (state) {
                case DRIVE_TO_SHOOT:
                    if (!drive.getFollower().isBusy()) {
                        state = State.DEPOSIT;
                    }
                    break;

                case DEPOSIT:
                    boolean depositDone = deposit.update();
                    if (depositDone) {
                        state = State.RETURN_HOME;
                        drive.getFollower().followPath(plan.returnPath);
                    }
                    break;

                case RETURN_HOME:
                    if (!drive.getFollower().isBusy()) {
                        state = State.DONE;
                    }
                    break;

                case DONE:
                    drive.setAutoDrive(0, 0, 0, true, 0);
                    shooter.setAutoRpm(0);
                    shooter.operate();
                    break;
            }

            // Telemetry
            addTelemetry(state, plan, drive, isRed, isAudienceSide);
            TelemetryHelper.update();

            sleep(20);
        }
    }

    private void addTelemetry(State state, SimpleAutoConfig.PathPlan plan,
                              Mecanum drive, boolean isRed, boolean isAudienceSide) {
        TelemetryHelper tele = new TelemetryHelper(this, true);

        Pose currentPose = drive.getFollower().getPose();

        tele.addLine("=== SIMPLE AUTO ===")
                .addData("State", "%s", state.name())
                .addData("Alliance", "%s %s", isRed ? "RED" : "BLUE",
                        isAudienceSide ? "AUD" : "DEPOT");

        if (state == State.DRIVE_TO_SHOOT || state == State.RETURN_HOME) {
            Pose target = (state == State.DRIVE_TO_SHOOT) ? plan.shootPose : plan.startPose;
            double dist = Math.hypot(
                    target.getX() - currentPose.getX(),
                    target.getY() - currentPose.getY()
            );
            tele.addData("Distance", "%.1f\"", dist);
        }

        tele.addData("PathBusy", "%b", true, drive.getFollower().isBusy());
    }
}
