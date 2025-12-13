package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.isAudienceSide;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.isRed;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.config.AutoConfig;
import org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig;
import org.firstinspires.ftc.teamcode.config.DecodeGameConfig;
import org.firstinspires.ftc.teamcode.config.TestAutoConfig;
import org.firstinspires.ftc.teamcode.managers.AutoManager;
import org.firstinspires.ftc.teamcode.managers.InventoryManager;
import org.firstinspires.ftc.teamcode.managers.PersistentBallState;
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

@Autonomous(name = "CHOOSE THSI ONE", group = "Testing")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutoConfig.isRed = TestAutoConfig.FORCE_RED;
        AutoConfig.isAudienceSide = TestAutoConfig.FORCE_AUDIENCE_SIDE;

        RobotHardware hw = new RobotHardware(this);
        GamepadMap map = new GamepadMap(this);

        Mecanum drive = new Mecanum(this, map);
        drive.init();

        hw.initIntake();
        Intake intake = new Intake(hw.getIntakeMotor(), this);

        hw.initShooter();
        Shooter shooter = new Shooter(hw.getShooterMotor(), hw.getShooterMotor1(),this);

        hw.initShooterYaw();
        ShooterYaw shooterYaw = new ShooterYaw(hw.getShooterYawMotor(), drive.getFollower(), isRed, this);

        hw.initSpindexer();
        Spindexer spindexer = new Spindexer(hw.getSpindexerServo(), this);

        hw.initSpindexColorSensors();
        SlotColorSensors slots = new SlotColorSensors(hw.getSpindexSensors(), this);

        hw.initTransfer();
        Transfer transfer = new Transfer(hw.getTransferServo1(), hw.getTransferServo2(), this);

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

        Pose startPose = DecodeGameConfig.startPose(isRed, isAudienceSide);
        Pose shootPose = DecodeGameConfig.shootPose(isRed);
        Pose finalPose = DecodeGameConfig.finalPose(isRed);

        drive.setStartingPose(startPose);
        drive.startAuto();

        AutoManager.Options options = new AutoManager.Options(
                TestAutoConfig.ENABLE_INTAKE,
                TestAutoConfig.ENABLE_DEPOSIT,
                TestAutoConfig.ENABLE_FINAL_MOVE
        );

        AutoManager auto = new AutoManager(
                drive, shooter, shooterYaw, spindexer, intake, transfer,
                slots, inv, isRed, shootPose, finalPose, uiLight, tele, options
        );

        boolean depositRoute = TestAutoConfig.RUN_DEPOSIT_ROUTE && TestAutoConfig.ENABLE_DEPOSIT;
        auto.start(depositRoute);

        while (opModeIsActive()) {
            auto.update();
            drive.operate();

            // Save ball state so teleop can pick it up
            PersistentBallState.saveFromModel(inv.getModel());

            TelemetryHelper.update();
            sleep(20);
        }
    }
}
