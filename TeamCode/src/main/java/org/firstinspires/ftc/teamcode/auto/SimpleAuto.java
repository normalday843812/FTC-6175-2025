package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_BLUE_AUDIENCE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_BLUE_DEPOT;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_RED_AUDIENCE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_RED_DEPOT;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.deposit.DepositController;
import org.firstinspires.ftc.teamcode.auto.motion.AllianceGoalHeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingController;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.Menu;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

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
        boolean isRed = menu.get("Alliance");
        boolean isAudienceSide = menu.get("Side");

        // Determine positions based on selection
        Pose startPose = pickStartPose(isRed, isAudienceSide);
        Pose shootPose = pickShootPose(isRed, isAudienceSide);
        Pose controlPoint = pickControlPoint(isRed, isAudienceSide);

        // Hardware and subsystems
        RobotHardware hw = new RobotHardware(this);
        hw.initLimeLight(100);
        hw.initShooter();
        hw.initSpindexer();
        hw.initTransfer();

        Mecanum drive = new Mecanum(this, null);
        drive.init();
        drive.setStartingPose(startPose);
        drive.startAuto();

        Shooter shooter = new Shooter(hw.getShooterMotor(), null, this);
        shooter.startAuto();

        Spindexer spindexer = new Spindexer(hw.getSpindexerServo(), null, this);
        spindexer.startAuto();

        Transfer transfer = new Transfer(hw.getTransferServo(), null, this);
        transfer.startAuto();

        // Motion setup
        HeadingController headingCtrl = new HeadingController();
        TelemetryHelper motionTele = new TelemetryHelper(this, true);
        MotionController motion = new MotionController(drive, headingCtrl, motionTele);

        Limelight3A limelight = hw.getLimelight();
        HeadingTarget goalTarget = new AllianceGoalHeadingTarget(limelight, isRed);

        // Deposit controller
        TelemetryHelper depositTele = new TelemetryHelper(this, true);
        DepositController deposit = new DepositController(
                shooter, spindexer, transfer, motion, goalTarget, isRed, depositTele
        );

        // Create paths
        Path toShootPath = new Path(new BezierCurve(startPose, controlPoint, shootPose));
        toShootPath.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        Path returnPath = new Path(new BezierCurve(shootPose, controlPoint, startPose));
        returnPath.setLinearHeadingInterpolation(shootPose.getHeading(), startPose.getHeading());

        if (isStopRequested()) return;

        waitForStart();

        State state = State.DRIVE_TO_SHOOT;
        drive.getFollower().followPath(toShootPath);

        while (opModeIsActive()) {
            drive.operate();
            transfer.operate();

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
                        drive.getFollower().followPath(returnPath);
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
            telemetry.addData("State", state.name());
            telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)",
                    drive.getFollower().getPose().getX(),
                    drive.getFollower().getPose().getY(),
                    Math.toDegrees(drive.getFollower().getPose().getHeading()));
            TelemetryHelper.update();

            sleep(20);
        }
    }

    private Pose pickStartPose(boolean isRed, boolean isAudienceSide) {
        if (isRed) return isAudienceSide ? START_RED_AUDIENCE : START_RED_DEPOT;
        return isAudienceSide ? START_BLUE_AUDIENCE : START_BLUE_DEPOT;
    }

    private Pose pickShootPose(boolean isRed, boolean isAudienceSide) {
        if (isRed && isAudienceSide) {
            return new Pose(96, 96, Math.toRadians(45));
        } else if (!isRed && isAudienceSide) {
            return new Pose(48, 96, Math.toRadians(135));
        } else if (isRed && !isAudienceSide) {
            return new Pose(96, 96, Math.toRadians(45));
        } else {
            return new Pose(48, 96, Math.toRadians(135));
        }
    }

    private Pose pickControlPoint(boolean isRed, boolean isAudienceSide) {
        if (isRed && isAudienceSide) {
            return new Pose(80, 86, 0);
        } else if (!isRed && isAudienceSide) {
            return new Pose(66, 85, 0);
        } else if (isRed && !isAudienceSide) {
            return new Pose(73, 138, 0);
        } else {
            return new Pose(71, 132, 0);
        }
    }
}