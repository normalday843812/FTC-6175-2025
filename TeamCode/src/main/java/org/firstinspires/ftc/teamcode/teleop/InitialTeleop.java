package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.isRed;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.config.DecodeGameConfig;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.config.UiLightConfig;
import org.firstinspires.ftc.teamcode.managers.InventoryManager;
import org.firstinspires.ftc.teamcode.managers.LightController;
import org.firstinspires.ftc.teamcode.managers.ShooterManager;
import org.firstinspires.ftc.teamcode.managers.TeleopSortManager;
import org.firstinspires.ftc.teamcode.managers.UiLight;
import org.firstinspires.ftc.teamcode.shooting.PolynomialRpmModel;
import org.firstinspires.ftc.teamcode.shooting.RpmModel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.RgbIndicator;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterYaw;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;
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
        Shooter shooter = new Shooter(hw.getShooterMotor(), hw.getShooterMotor1(), map, this);

        hw.initShooterYaw();
        ShooterYaw shooterYaw = new ShooterYaw(hw.getShooterYawMotor(),
                aprilTag, map, drive.getFollower(), this);

        hw.initHood();
        Hood hood = new Hood(hw.getHoodServo(), map, this);

        hw.initSpindexer();
        Spindexer spindexer = new Spindexer(hw.getSpindexerServo(), map, this);

        hw.initSpindexColorSensors();
        SlotColorSensors slots = new SlotColorSensors(hw.getSlotColor0(),
                hw.getSlotColor1(), hw.getSlotColor2(), this);
        
        // link slots to spindexer for color-based positioning
        spindexer.setColorSlots(slots);

        hw.initTransfer();
        Transfer transfer = new Transfer(hw.getTransferServo1(),
                hw.getTransferServo2(), map, this);

        hw.initRgbIndicator();
        RgbIndicator rgbIndicator = new RgbIndicator(hw.getRgbIndicator());
        UiLight ui = new UiLight(rgbIndicator);
        LightController light = new LightController(ui, shooter, shooterYaw, intake);

        InventoryManager inv = new InventoryManager();
        TeleopSortManager teleopSortManager =
                new TeleopSortManager(map, intake, spindexer, transfer, slots, inv);

        ui.setBase(UiLightConfig.UiState.READY);

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
        shooterManager.setLimits(ShooterConfig.IDLE_RPM,
                ShooterConfig.MAX_RPM);

        Pose goal = DecodeGameConfig.shootPose(isRed);
        Limelight3A limelight = hw.getLimelight();

        while (opModeIsActive()) {
            map.update();

            if (map.shooterManagerToggle) managerEnabled = !managerEnabled;
            shooterManager.setEnabled(managerEnabled);

            Pose current = drive.getFollower().getPose();
            shooterManager.update(current, goal, limelight);

            transfer.operate();
            slots.update();
            aprilTag.update();
            drive.operate();
            intake.operate();
            shooter.operate();
            hood.operate();
            spindexer.operate();
            shooterYaw.operate();

            teleopSortManager.update();

            TelemetryHelper.update();
        }
    }
}
