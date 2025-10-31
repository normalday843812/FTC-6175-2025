package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.isAudienceSide;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.isRed;
import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.pickShootPose;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.IDLE_RPM;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.MAX_RPM;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.shooting.PolynomialRpmModel;
import org.firstinspires.ftc.teamcode.shooting.RpmModel;
import org.firstinspires.ftc.teamcode.shooting.ShooterManager;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterYaw;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.vision.LLAprilTag;

@TeleOp
public class InitialTeleop extends LinearOpMode {
    boolean managerEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        GamepadMap map = new GamepadMap(this);

        // Drive
        Mecanum drive = new Mecanum(this, map);
        drive.init();

        // Limelight
        hw.initLimeLight(100);
        hw.setLimelightPipeline(0);
        LLAprilTag aprilTag = new LLAprilTag(hw.getLimelight(), this);

        // Subsystems
        hw.initIntake();
        Intake intake = new Intake(hw.getIntakeMotor(), map, this);

        hw.initShooter();
        Shooter shooter = new Shooter(hw.getShooterMotor(), map, this);

        hw.initShooterYaw();
        ShooterYaw shooterYaw = new ShooterYaw(hw.getShooterYawMotor(), aprilTag, isRed,
                map, this);

        hw.initHood();
        Hood hood = new Hood(hw.getHoodServo(), map, this);

        hw.initSpindexer();
        Spindexer spindexer = new Spindexer(hw.getSpindexerServo(), map, this);

        hw.initTransfer();
        Transfer transfer = new Transfer(hw.getTransferServo1(),
                hw.getTransferServo2(), map, this);

        hw.initIntakeColorSensor();
        IntakeColorSensor intakeColorSensor = new IntakeColorSensor(hw.getIntakeColorSensor(), this);

        if (isStopRequested()) return;
        waitForStart();

        drive.startTeleop();
        intake.startTeleop();
        shooter.startTeleop();
        hood.startTeleop();
        spindexer.startTeleop();
        transfer.startTeleop();
        shooterYaw.startTeleop();

        RpmModel model = new PolynomialRpmModel();
        ShooterManager shooterManager = new ShooterManager(shooter, model, this);
        shooterManager.setLimits(IDLE_RPM,
                MAX_RPM);

        Pose goal = pickShootPose(isRed, isAudienceSide);
        Limelight3A limelight = hw.getLimelight();

        while (opModeIsActive()) {
            map.update();

            if (map.shooterManagerToggle) managerEnabled = !managerEnabled;
            shooterManager.setEnabled(managerEnabled);

            Pose current = drive.getFollower().getPose();
            shooterManager.update(current, goal, limelight);

            transfer.operate();
            intakeColorSensor.update();
            aprilTag.update();
            drive.operate();
            intake.operate();
            shooter.operate();
            hood.operate();
            spindexer.operate();
            shooterYaw.operate();

            TelemetryHelper.update();
        }
    }
}
