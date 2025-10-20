package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.vision.LLAprilTag;

@TeleOp
public class InitialTeleop extends LinearOpMode {
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

        hw.initHood();
        Hood hood = new Hood(hw.getHoodServo(), map, this);

        hw.initSpindexer();
        Spindexer spindexer = new Spindexer(hw.getSpindexerServo(), map, this);

        hw.initIntakeColorSensor();
        IntakeColorSensor intakeColorSensor = new IntakeColorSensor(hw.getIntakeColorSensor(), this);

        if (isStopRequested()) return;
        waitForStart();

        drive.startTeleop();

        while (opModeIsActive()) {
            map.update();
            intakeColorSensor.update();
            aprilTag.update();
            drive.operate();
            intake.operate();
            shooter.operate();
            hood.operate();
            spindexer.operate();
            TelemetryHelper.update();
        }
    }
}
