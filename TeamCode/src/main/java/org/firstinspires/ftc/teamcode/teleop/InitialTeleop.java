package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.localisation.StateEstimator;
import org.firstinspires.ftc.teamcode.localisation.StateEstimatorTwoWheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.MecanumTwoOdom;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;

@Disabled
@TeleOp
public class InitialTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        AprilTagLocalizer aprilTagLocalizer = new AprilTagLocalizer(
                this
        );

        hw.initWebcam();
        hw.initVision(hw.getWebcam1(), aprilTagLocalizer.getAprilTag());

        aprilTagLocalizer.initAprilTag();

        hw.initPinpoint();
        StateEstimator state = new StateEstimator(
                this,
                hw.getPinpoint(),
                aprilTagLocalizer
        );
//        StateEstimatorTwoWheel state = new StateEstimatorTwoWheel(
//                this,
//                hw.getOdoParallel(),
//                hw.getOdoPerp(),
//                hw.getIMU(),
//                aprilTagLocalizer
//        );
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
//        MecanumTwoOdom drive = new MecanumTwoOdom(
//                hw.getFrontLeft(),
//                hw.getBackLeft(),
//                hw.getFrontRight(),
//                hw.getBackRight(),
//                state,
//                map,
//                this
//        );

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
            intake.OperateIntake();
            shooter.OperateShooter();
            telemetry.update();
        }

        hw.closeVision();
    }
}
