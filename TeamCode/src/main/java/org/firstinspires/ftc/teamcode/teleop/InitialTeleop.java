package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.isRed;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.managers.InventoryManager;
import org.firstinspires.ftc.teamcode.managers.LightController;
import org.firstinspires.ftc.teamcode.managers.PersistentBallState;
import org.firstinspires.ftc.teamcode.managers.TeleopManager;
import org.firstinspires.ftc.teamcode.managers.UiLight;
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

@TeleOp
public class InitialTeleop extends LinearOpMode {
    boolean managerEnabled = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(true);

        RobotHardware hw = new RobotHardware(this);

        GamepadMap map = new GamepadMap(this);

        // Drive
        Mecanum drive = new Mecanum(this, map);
        drive.init();

        // Subsystems
        hw.initIntake();
        Intake intake = new Intake(hw.getIntakeMotor(), this);

        hw.initShooter();
        Shooter shooter = new Shooter(hw.getShooterMotor(), hw.getShooterMotor1(), this);

        hw.initShooterYaw();
        ShooterYaw shooterYaw = new ShooterYaw(hw.getShooterYawMotor(), drive.getFollower(), isRed, this);

        hw.initHood();
        Hood hood = new Hood(hw.getHoodServo(), map, this);

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
        LightController light = new LightController(ui, shooter, shooterYaw, intake);

        InventoryManager inventoryManager = new InventoryManager();

        // Load persistent ball state from auto (if available)
        if (PersistentBallState.isInitialized()) {
            PersistentBallState.loadIntoModel(inventoryManager.getModel());
            telemetry.addLine("Loaded ball state from auto");
            telemetry.addData("Balls", PersistentBallState.getBallCount());
        } else {
            telemetry.addLine("No ball state from auto - starting fresh");
        }
        telemetry.update();

        TeleopManager teleopManager =
                new TeleopManager(intake, shooter, shooterYaw, spindexer, transfer, slots, inventoryManager, ui, this);

        if (isStopRequested()) return;
        waitForStart();

        drive.startTeleop();
        shooter.start();
        hood.startTeleop();

        spindexer.start();
        transfer.start();
        shooterYaw.start();

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
            hood.operate();
            teleopManager.update(map);
            light.update(true);

            // Save ball state periodically so it persists
            PersistentBallState.saveFromModel(inventoryManager.getModel());

            TelemetryHelper.update();
        }
    }
}
