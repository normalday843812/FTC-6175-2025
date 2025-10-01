package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.localisation.StateEstimator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;

@TeleOp
public class ShooterTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);
        hw.initHardware();

//        AprilTagLocalizer aprilTagLocalizer = new AprilTagLocalizer(
//                this
//        );
//
//        aprilTagLocalizer.initAprilTag();
//
//        StateEstimator state = new StateEstimator(
//                this,
//                hw.getPinpoint(),
//                aprilTagLocalizer
//        );
        GamepadMap map = new GamepadMap(this);
//
//        Mecanum drive = new Mecanum(
//                hw.getFrontLeft(),
//                hw.getBackLeft(),
//                hw.getFrontRight(),
//                hw.getBackRight(),
//                state,
//                map,
//                this
//        );

//        Intake intake = new Intake(
//                hw.getIntakeMotor(),
//                map,
//                this
//        );

        Shooter shooter = new Shooter(
            hw.getShooterMotor(),
            map,
            this
        );

//        hw.initVision(aprilTagLocalizer.getAprilTag());

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {
            map.update();
//            state.update();
//            drive.operateMecanum();
            shooter.OperateShooter();
//            intake.OperateIntake();
            telemetry.update();
        }

//        hw.closeVision();
    }
}
