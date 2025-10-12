package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.localisation.StateEstimator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizerLimelight;

@TeleOp
public class InitialTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        AprilTagLocalizerLimelight aprilTagLocalizer = new AprilTagLocalizerLimelight(
                hw.getLimelight()
        );

        hw.initLimeLight(100);
        hw.setLimelightPipeline(0);

        hw.initPinpoint();
        StateEstimator state = new StateEstimator(
                this,
                hw.getPinpoint(),
                aprilTagLocalizer
        );
        GamepadMap map = new GamepadMap(this);

        hw.initDriveMotors();
        Mecanum drive = new Mecanum(
                hw.getFrontLeft(),
                hw.getBackLeft(),
                hw.getFrontRight(),
                hw.getBackRight(),
                state,
                map,
                this
        );

        hw.initIntake();
        Intake intake = new Intake(
                hw.getIntakeMotor(),
                map,
                this
        );

        hw.initShooter();
        Shooter shooter = new Shooter(
                hw.getShooterMotor(),
                map,
        this
        );

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {
            map.update();
            state.update();
            drive.operateMecanum();
            intake.operateIntake();
            shooter.operateShooter();
            telemetry.update();
        }
    }
}
