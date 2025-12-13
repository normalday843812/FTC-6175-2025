package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.isAudienceSide;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.isRed;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig;
import org.firstinspires.ftc.teamcode.config.DecodeGameConfig;
import org.firstinspires.ftc.teamcode.managers.AutoManager;
import org.firstinspires.ftc.teamcode.managers.InventoryManager;
import org.firstinspires.ftc.teamcode.managers.PersistentBallState;
import org.firstinspires.ftc.teamcode.managers.UiLight;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.RgbIndicator;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterYaw;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.Menu;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@Autonomous(name = "CHOOSE THIS AUTO", group = "Pedro")
public class SimpleAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);
        GamepadMap map = new GamepadMap(this);

        // Subsystems
        Mecanum drive = new Mecanum(this, map);
        drive.init();

        hw.initIntake();
        Intake intake = new Intake(hw.getIntakeMotor(), this);

        hw.initShooter();
        Shooter shooter = new Shooter(hw.getShooterMotor(), hw.getShooterMotor1(),this);

        hw.initSpindexer();
        Spindexer spindexer = new Spindexer(hw.getSpindexerServo(), this);

        hw.initSpindexColorSensors();
        SlotColorSensors slots = new SlotColorSensors(hw.getSpindexSensors(), this);

        hw.initTransfer();
        Transfer transfer = new Transfer(hw.getTransferServo1(), hw.getTransferServo2(), this);

        hw.initRgbIndicator();
        RgbIndicator rgbIndicator = new RgbIndicator(hw.getRgbIndicator());

        UiLight uiLight = new UiLight(rgbIndicator);

        // Managers
        InventoryManager inv = new InventoryManager();
        TelemetryHelper tele = new TelemetryHelper(this, AutoUnifiedConfig.TELEMETRY_ENABLED);

        // Menu
        Menu menu = new Menu(this)
                .add(new Menu.Item("Alliance", "Red (B)", () -> gamepad1.b, "Blue (X)", () -> gamepad1.x, isRed))
                .add(new Menu.Item("Side", "Audience (Y)", () -> gamepad1.y, "Depot (A)", () -> gamepad1.a, isAudienceSide))
                .add(new Menu.Item("Deposit?", "Yes (RB)", () -> gamepad1.right_bumper, "No (LB)", () -> gamepad1.left_bumper, true));
        menu.showUntilStart();
        isRed = menu.get("Alliance");
        isAudienceSide = menu.get("Side");
        boolean depositRoute = menu.get("Deposit?");

        if (isStopRequested()) return;
        waitForStart();
        hw.initShooterYaw();
        ShooterYaw shooterYaw = new ShooterYaw(hw.getShooterYawMotor(), drive.getFollower(), isRed, this);

        Pose startPose = DecodeGameConfig.startPose(isRed, isAudienceSide);
        Pose shootPose = DecodeGameConfig.shootPose(isRed);
        Pose finalPose = DecodeGameConfig.finalPose(isRed);

        drive.setStartingPose(startPose);
        drive.startAuto();

        AutoManager auto = new AutoManager(
                drive, shooter, shooterYaw, spindexer, intake, transfer,
                slots, inv, isRed, shootPose, finalPose, uiLight, tele
        );
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
