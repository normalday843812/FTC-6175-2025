package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.isAudienceSide;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.isRed;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.motion.AllianceGoalHeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.config.AutoConfig;
import org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig;
import org.firstinspires.ftc.teamcode.config.DecodeGameConfig;
import org.firstinspires.ftc.teamcode.config.TestAutoConfig;
import org.firstinspires.ftc.teamcode.managers.AutoManager;
import org.firstinspires.ftc.teamcode.managers.InventoryManager;
import org.firstinspires.ftc.teamcode.managers.UiLight;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.RgbIndicator;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterYaw;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.vision.LLAprilTag;

@Autonomous(name = "Test Auto", group = "Testing")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutoConfig.isRed = TestAutoConfig.FORCE_RED;
        AutoConfig.isAudienceSide = TestAutoConfig.FORCE_AUDIENCE_SIDE;

        RobotHardware hw = new RobotHardware(this);
        GamepadMap map = new GamepadMap(this);

        Mecanum drive = new Mecanum(this, map);
        drive.init();

        hw.initLimeLight(100);
        LLAprilTag ll = new LLAprilTag(hw.getLimelight(), this);

        hw.initIntake();
        Intake intake = new Intake(hw.getIntakeMotor(), null, this);

        hw.initShooter();
        Shooter shooter = new Shooter(hw.getShooterMotor(), hw.getShooterMotor1(), map,this);

        hw.initShooterYaw();
        ShooterYaw shooterYaw = new ShooterYaw(hw.getShooterYawMotor(), ll, null, drive.getFollower(), this);

        hw.initSpindexer();
        Spindexer spindexer = new Spindexer(hw.getSpindexerServo(), null, this);

        hw.initSpindexColorSensors();
        SlotColorSensors slots = new SlotColorSensors(hw.getSpindexSensors(), this);

        hw.initTransfer();
        Transfer transfer = new Transfer(hw.getTransferServo1(), hw.getTransferServo2(), null, this);

        RgbIndicator rgbIndicator;
        UiLight uiLight = null;

        if (TestAutoConfig.USE_UI_LIGHT) {
            hw.initRgbIndicator();
            rgbIndicator = new RgbIndicator(hw.getRgbIndicator());
            uiLight = new UiLight(rgbIndicator);
        }

        InventoryManager inv = new InventoryManager();
        TelemetryHelper tele = new TelemetryHelper(this, AutoUnifiedConfig.TELEMETRY_ENABLED);

        if (isStopRequested()) return;
        waitForStart();

        MotionController motion = new MotionController(drive, tele);
        AllianceGoalHeadingTarget heading = new AllianceGoalHeadingTarget(ll, isRed);

        Pose startPose = DecodeGameConfig.startPose(isRed, isAudienceSide);
        Pose shootPose = DecodeGameConfig.shootPose(isRed);
        Pose finalPose = DecodeGameConfig.finalPose(isRed);

        drive.setStartingPose(startPose);
        drive.startAuto();

        AutoManager.Options options = new AutoManager.Options(
                TestAutoConfig.ENABLE_PATTERN_SEEK,
                TestAutoConfig.ENABLE_INTAKE,
                TestAutoConfig.ENABLE_DEPOSIT,
                TestAutoConfig.ENABLE_FINAL_MOVE,
                TestAutoConfig.USE_COLOR_SENSORS
        );

        AutoManager auto = new AutoManager(
                drive, motion, shooter, shooterYaw, spindexer, intake, transfer,
                slots, inv, heading, isRed, shootPose, finalPose, uiLight, tele, options
        );

        boolean depositRoute = TestAutoConfig.RUN_DEPOSIT_ROUTE && TestAutoConfig.ENABLE_DEPOSIT;
        auto.start(depositRoute);

        while (opModeIsActive()) {
            ll.update();
            auto.update();
            drive.operate();
            TelemetryHelper.update();
            sleep(20);
        }
    }
}